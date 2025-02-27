#!/usr/bin/env python3

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
from pathlib import Path

import multiprocessing

logger = logging.getLogger(__name__)


# Be sure to use system codesign and not one embedded into the conda env
CODESIGN_FULL_PATH = "/usr/bin/codesign"

# Entitlements declaration
PYAPP_ENTITLEMENTS = Path(__file__).parent / "pythonapp.entitlements"
CCAPP_ENTITLEMENTS = Path(__file__).parent / "ccapp.entitlements"
HARDENED_CCAPP_ENTITLEMENTS = Path(__file__).parent / "hdapp.entitlements"


class CCSignBundleConfig:
    install_path: Path
    bundle_abs_path: Path
    cc_bin_path: Path

    signature: str
    identifier: str

    embed_python: bool

    def __init__(self, install_path: Path, signature: str, identifier: str, embed_python: bool) -> None:
        """Construct a configuration.

        Args:
        ----
            install_path (Path):  Path where CC is "installed".
            signature (str): Signature to use to sign binaries in the bundle (`codesign -s` option).
            identifier (str): Identifier to use to sign binaries in the bundle (`codesign -i` option).
            embed_python (bool): Whether or not Python is embedded in the package.

        """
        self.install_path = install_path
        self.bundle_abs_path = install_path / "CloudCompare" / "CloudCompare.app"
        self.cc_bin_path = self.bundle_abs_path / "Contents" / "MacOS" / "CloudCompare"
        self.embed_python = embed_python

        self.signature = signature
        self.identifier = identifier

        if self.embed_python:
            self.embedded_python_rootpath = self.bundle_abs_path / "Contents" / "Resources" / "python"
            self.embedded_python_path = self.embedded_python_rootpath / "bin"
            self.embedded_python_binary = self.embedded_python_path / "python"
            self.embedded_python_libpath = self.embedded_python_rootpath / "lib"


class CCSignBundle:
    config: CCSignBundleConfig

    def __init__(self, config: CCSignBundleConfig) -> None:
        """Construct a CCSingBundle.

        Args:
        ----
            config (CCSignBundleConfig): The configuration.

        """
        self.config = config

    @staticmethod
    def _remove_signature(path: Path) -> None:
        """Remove signature of a binary file.

        Call `codesign` utility via a subprocess

        Args:
        ----
            path (Path): The path to the file to sign.

        """
        subprocess.run([CODESIGN_FULL_PATH, "--remove-signature", str(path)], stdout=subprocess.PIPE, check=True)

    def _add_signature(self, path: Path) -> None:
        """Sign a binary file.

        Call `codesign` utility via a subprocess. The signature stored
        into the `config` object is used to sign the binary.

        Args:
        ----
            path (Path): The path to the file to sign.

        """
        subprocess.run(
            [CODESIGN_FULL_PATH, "-s", self.config.signature, "-f", "--timestamp", str(path)],
            stdout=subprocess.PIPE,
            check=True,
        )

    def _add_entitlements(self, path: Path, entitlements: Path) -> None:
        """Sign a binary file with some specific entitlements.

        Call `codesign` utility via a subprocess. The signature and the
        identifier stored into the `config` object are used to sign the binary.

        Args:
        ----
            path (Path): The path to the file to sign.
            entitlements (Path): Path to a file that contains entitlements

        """
        subprocess.run(
            [
                CODESIGN_FULL_PATH,
                "-s",
                self.config.signature,
                "-f",
                "--timestamp",
                "-i",
                self.config.identifier,
                "--entitlements",
                str(entitlements),
                str(path),
            ],
            stdout=subprocess.PIPE,
            check=True,
        )

    def sign(self) -> int:
        """Process and sign the bundle.

        Returns
        -------
            (int) : Error code at the end of the process.

        """
        # collect all libs in the bundle
        # split set to have 100% Python lib in one set and other libs in another set
        # TODO: Not sure the split is really usefull
        logger.info("Collect libs in the bundle")
        so_generator = self.config.bundle_abs_path.rglob("*.so")
        dylib_generator = self.config.bundle_abs_path.rglob("*.dylib")
        all_libs = set(list(so_generator) + list(dylib_generator))
        if self.config.embed_python:
            python_libs = set(filter(lambda p: p.is_relative_to(self.config.embedded_python_rootpath), all_libs))
            cc_app_libs = all_libs - python_libs

        logger.info("--- Total # lib in the bundle %i", len(all_libs))

        if self.config.embed_python:
            logger.info("--- Total # lib in the python sub system %i", len(python_libs))
            logger.info("--- Total # lib in the CC sub system (framework and plugins) %i", len(cc_app_libs))

        logger.info("Remove old signatures")
        
        # create the process pool
        process_pool = multiprocessing.Pool()
        # Remove signature in all embedded libs
        process_pool.map(CCSignBundle._remove_signature, all_libs)

        # Remove CC signature as well
        CCSignBundle._remove_signature(self.config.cc_bin_path)

        if self.config.embed_python:
            CCSignBundle._remove_signature(self.config.embedded_python_binary)

            logger.info("Sign Python dynamic libraries")
            process_pool.map(self._add_signature, python_libs)

            logger.info("Add entitlements to Python binary")
            self._add_entitlements(self.config.embedded_python_binary, PYAPP_ENTITLEMENTS)

            logger.info("Sign CC dynamic libraries")
            process_pool.map(self._add_signature, cc_app_libs)

            logger.info("Add entitlements to CC binary")
            self._add_entitlements(self.config.cc_bin_path, CCAPP_ENTITLEMENTS)

            logger.warning("Add entitlements to CC bundle (CloudCompare.app)")
            self._add_entitlements(self.config.bundle_abs_path, CCAPP_ENTITLEMENTS)

        else:
            logger.info("Sign CC dynamic libraries")
            process_pool.map(self._add_signature, cc_app_libs)

            logger.info("Add entitlements to CC binary")
            # TODO: the original shell script sign the binary AND add entitlements...
            # is it necessary?
            self._add_entitlements(self.config.cc_bin_path, HARDENED_CCAPP_ENTITLEMENTS)

            logger.warning("Add entitlements to CC bundle (CloudCompare.app)")
            self._add_entitlements(self.config.bundle_abs_path, HARDENED_CCAPP_ENTITLEMENTS)
        process_pool.close()
        return 0


if __name__ == "__main__":
    # configure logger
    formatter = "CCSignBundle::%(levelname)-6s:: %(message)s"
    logging.basicConfig(level=logging.INFO, format=formatter)
    std_handler = logging.StreamHandler()

    # CLI parser
    parser = argparse.ArgumentParser("CCSignBundle")
    parser.add_argument(
        "install_path",
        help="Path where the CC application is installed (CMake install prefix)",
        type=Path,
    )

    parser.add_argument(
        "--signature",
        help="Signature to use for code signing (or will use CC_BUNDLE_SIGN var)",
        type=str,
    )

    parser.add_argument(
        "--identifier",
        help="Identifier to use for code signing",
        type=str,
        default="fr.openfields.CloudCompare",
    )

    parser.add_argument(
        "--embed_python",
        help="Whether embedding python or not",
        action="store_true",
    )
    arguments = parser.parse_args()

    if arguments.signature is not None:
        signature = arguments.signature
    else:
        signature = os.environ.get("CC_BUNDLE_SIGN")
        if signature is None:
            logger.error(
                "CC_BUNDLE_SIGN variable is undefined. Please define it or use the `signature` argument in the CLI",
            )
            sys.exit(1)

    logger.info("Identifier: %s", arguments.identifier)
    logger.debug("Signature: %s", signature)  # Could be dangerous to display this

    try:
        Path(CODESIGN_FULL_PATH).exists()
    except Exception:
        logger.exception("Unable to find codesign binary on this computer")
        sys.exit(1)

    config = CCSignBundleConfig(
        arguments.install_path,
        signature,
        arguments.identifier,
        arguments.embed_python,
    )

    sign_bundle = CCSignBundle(config)
    sys.exit(sign_bundle.sign())
