#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import logging
import os
import shutil
import subprocess
import sys
from pathlib import Path

logger = logging.getLogger(__name__)


class CCAppBundleConfig:
    CloudCompareApp: str
    output_dependencies: bool
    embed_python: bool

    cc_bin_path: str
    extra_pathlib: str | None = None
    frameworks_path: str
    plugin_path: str
    embedded_python_rootpath: str

    python_version: str  # pythonMajor.Minor
    base_python_binary: str  # "prefix/bin/python"
    base_python_libs: str  # "prefix/lib/pythonMajor.Minor"

    def __init__(
        self,
        install_path: str,
        extra_pathlib: str,
        output_dependencies: bool,
        embed_python: bool,
    ) -> None:
        """Construct a configuration.

        Args:
        ----
            install_path (str): string representation of the path where CC is "installed".
            extra_pathlib (str): string representation of a path where additional libs can be found.
            output_dependencies (bool): boolean that control the level of debug. If true some extra
            files will be created (macos_bundle_warnings.json macos_bundle_dependencies.json).

        """
        self.output_dependencies = output_dependencies
        self.bundle_abs_path = install_path + "/CloudCompare/CloudCompare.app"
        self.cc_bin_path = self.bundle_abs_path + "/Contents/MacOS/CloudCompare"
        self.extra_pathlib = extra_pathlib
        self.frameworks_path = os.path.abspath(
            self.bundle_abs_path + "/Contents/Frameworks",
        )
        self.plugin_path = os.path.abspath(self.bundle_abs_path + "/Contents/PlugIns")

        # if we want to embed Python we populate the needed variables
        self.embed_python = embed_python
        if embed_python:
            self._query_python()
            self.embedded_python_rootpath = self.bundle_abs_path + "/Contents/Resources/python"
            self.embedded_python_path = os.path.abspath(
                os.path.join(self.embedded_python_rootpath, "bin"),
            )
            self.embedded_python_binary = os.path.abspath(
                os.path.join(self.embedded_python_path, "python"),
            )
            self.embedded_python_libpath = os.path.abspath(
                os.path.join(self.embedded_python_rootpath, "lib"),
            )
            self.embedded_python_lib = os.path.abspath(
                os.path.join(
                    self.embedded_python_libpath,
                    "python" + self.python_version,
                ),  # TODO: param
            )
            self.embedded_python_site_package = os.path.abspath(
                os.path.join(self.embedded_python_lib, "site-packages"),
            )

    def __str__(self) -> str:
        """Return a string representation of the class."""
        res = (
            f"--- Frameworks path: {self.frameworks_path} \n"
            f" --- plugin path: {self.plugin_path} \n"
            f" --- embeddedPythonPath: {self.embedded_python_path} \n"
            f" --- embeddedPython:  {self.embedded_python_binary} \n"
            f" --- embeddedPythonLibPath: {self.embedded_python_libpath} \n"
            f" --- embeddedPythonLib: {self.embedded_python_lib} \n"
            f" --- embeddedPythonSiteLibs: {self.embedded_python_site_package} \n"
        )
        return res

    def _query_python(self):
        """Query for python paths and configuration."""
        self.python_version = f"{sys.version_info.major}.{sys.version_info.minor}"
        self.base_python_binary = f"{sys.exec_prefix}/bin/python"
        self.base_python_libs = f"{sys.exec_prefix}/lib/python{self.python_version}"


class CCBundler:
    config: CCAppBundleConfig

    # dictionary of lib dependencies : key depends on (list of libs) (not recursive)
    dependencies: dict[str, list[str]] = dict()
    warnings: dict[str, list[str]] = dict()

    def __init__(self, config: CCAppBundleConfig) -> None:
        """Construct a CCBundler object"""
        self.config = config

    def bundle(self) -> None:
        """Bundle the dependencies into the .app"""
        if config.embed_python:
            self._embed_python()

        libs_found, libs_ex_found, libs_in_plugins = self._collect_dependencies()
        self._embed_libraries(libs_found, libs_ex_found, libs_in_plugins)

        # output debug files if needed
        if self.config.output_dependencies:
            logger.info("write debug files (macos_bundle_dependencies.json and macos_bundle_warnings.json)")
            with open(
                os.path.join(os.getcwd(), "macos_bundle_dependencies.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump(self.dependencies, f, sort_keys=True, indent=4)

            with open(
                os.path.join(os.getcwd(), "macos_bundle_warnings.json"),
                "w",
                encoding="utf-8",
            ) as f:
                json.dump(self.warnings, f, sort_keys=True, indent=4)

    def _get_lib_dependencies(self, mainlib: str) -> tuple[list[str], list[str]]:
        """List dependencies of mainlib (using otool -L).

        We only look for dependencies with @rpath and @executable_path.
        We consider @executable_path being relative to the CloudCompare executable.
        We keep record and debug /usr and /System for debug purposes.

        Args:
        ----
            mainlib (str): string representation of the path to a binary (lib, executable)

        Returns:
        -------
            libs (list[str]): lib @rpath ou @executable_path
            lib_ex (list[str]): lib @executable_path

        """
        libs = []
        lib_ex = []
        warning_libs = []
        with subprocess.Popen(["otool", "-L", mainlib], stdout=subprocess.PIPE) as proc:
            lines = proc.stdout.readlines()
            logger.debug(mainlib)
            lines.pop(0)  # Drop the first line as it contains the name of the lib / binary
            # now first line is LC_ID_DYLIB (should be @rpath/libname)
            for line in lines:
                vals = line.split()
                if len(vals) < 2:
                    continue
                pathlib = vals[0].decode()
                logger.debug(f"->pathlib: {pathlib}")
                if pathlib == self.config.extra_pathlib:
                    logger.info(f"{mainlib} lib from additional extra pathlib")
                    libs.append(pathlib)
                    continue
                dirs = pathlib.split("/")
                # TODO: should be better with startswith
                # we are likely to have only @rpath values
                if dirs[0] == "@rpath":
                    libs.append(dirs[1])
                elif dirs[0] == "@loader_path":
                    logger.warning(
                        f"{mainlib} declares a dependencies with @loader_path, this won't be resolved",
                    )
                elif dirs[0] == "@executable_path":
                    logger.warning(
                        f"{mainlib} declares a dependencies with @executable_path",
                    )
                    # TODO: check if mainlib is in the bundle, to be sure executable path is relaltive to the application
                    lib_ex.append(
                        (
                            os.path.basename(mainlib),
                            pathlib.removeprefix("@executable_path/"),
                        ),
                    )
                elif (dirs[1] != "usr") and (dirs[1] != "System"):
                    logger.warning(f"{mainlib} depends on  undeclared pathlib: {pathlib}")
            self.warnings[mainlib] = warning_libs
            self.dependencies[os.path.basename(mainlib)] = libs
        return libs, lib_ex

    @staticmethod
    def _get_rpath(binary_path: str) -> list[str]:
        """Retrieve paths stored in LC_RPATH part of the binary.

        Paths are expected to be in the form @loader_path/xxx, @executable_path/xxx, or even abs and relative Paths

        Args:
        ----
            binary_path (str): string representation of the path to a binary (lib, executable)

        Returns:
        -------
        list[str]: rpath list

        """
        rpaths = []
        with subprocess.Popen(["otool", "-l", binary_path], stdout=subprocess.PIPE) as proc:
            lines = proc.stdout.readlines()
            for line in lines:
                res = line.decode()
                vals = res.split()
                if len(vals) > 1 and vals[0] == "path":
                    rpaths.append(vals[1])
        return rpaths

    @staticmethod
    def _convert_rpaths(binary_path: str, rpaths: list[str]) -> list[str]:
        """Convert rpaths to absolute paths.

        Given a path to a binary (lib, executable) and a list of rpaths, resolve rpaths
        and append binary_path to them in order to create putative aboslute path to this binary

        Args:
        ----
            binary_path (str): string representation of the path to a binary (lib, executable)

            rpaths (list[str]): string representation of rpath list

        Returns:
        -------
            list[str]: string representation of putative full path to the binary

        """
        dirname_binary = os.path.dirname(binary_path)
        abs_paths = []
        for rpath in rpaths:
            if "@loader_path" in rpath:
                vals = rpath.split("/")
                abs_path = dirname_binary
                if len(vals) > 1:
                    abs_path = os.path.abspath(os.path.join(abs_path, "/".join(vals[1:])))
            else:
                # TODO: test if it's an aboslute path
                abs_path = os.path.abspath(rpath)
            abs_paths.append(abs_path)
        return abs_paths

    def _copy_python_env(self) -> None:
        """Copy python environment.

        Ideally this should be handled by CCPython-Runtime CMake script like in Windows.
        """
        logger.info("Python: copy distribution in package")
        try:
            os.makedirs(self.config.embedded_python_path)
            os.makedirs(self.config.embedded_python_libpath)
        except OSError:
            logger.error(
                "Python dir already exists in bundle, please clean your bundle and rerun this script",
            )
            sys.exit(1)
        shutil.copytree(self.config.base_python_libs, self.config.embedded_python_lib)
        shutil.copy2(self.config.base_python_binary, self.config.embedded_python_binary)

    def _embed_python(self) -> None:
        """Embed python distribution dependencies in site-packages.

        It copies the pyhton target distribution in the `.app` bundle
        and then it collect dependencies and rewrites rpaths
        of all the binaries/libraries found inside the distribution's tree.
        """
        libs_to_check = [self.config.embedded_python_binary]

        # results
        libs_found = set()
        lib_ex_found = set()
        python_libs = set()  # Lib in python dir

        self._copy_python_env()
        # --- enumerate all libs inside the dir
        for root, _, files in os.walk(self.config.embedded_python_lib):
            for name in files:
                ext = os.path.splitext(name)[1]
                if ext in (".dylib", ".so"):
                    library = os.path.join(self.config.embedded_python_lib, root, name)
                    libs_to_check.append(library)
                    python_libs.add(library)

        logger.info(
            f"number of libs (.so and .dylib) in embedded Python: {len(python_libs)}",
        )

        while len(libs_to_check):
            lib2check = libs_to_check.pop(0)

            if lib2check in libs_found:
                continue

            libs_found.add(lib2check)

            libs, libex = self._get_lib_dependencies(lib2check)
            lib_ex_found.update(libex)

            rpaths = CCBundler._get_rpath(lib2check)

            abs_rpaths = CCBundler._convert_rpaths(lib2check, rpaths)
            if self.config.extra_pathlib not in abs_rpaths:
                abs_rpaths.append(self.config.extra_pathlib)

            for lib in libs:
                if lib[0] == "/":
                    if lib not in libs_to_check and lib not in libs_found:
                        libs_to_check.append(lib)
                else:
                    for abs_rp in abs_rpaths:
                        abs_lib = os.path.join(abs_rp, lib)
                        if os.path.isfile(abs_lib):
                            if abs_lib not in libs_to_check and abs_lib not in libs_found:
                                libs_to_check.append(abs_lib)
                            break

        logger.info(f"lib_ex_found to add to Frameworks: {len(lib_ex_found)}")
        logger.info(f"libs_found to add to Frameworks: {len(libs_found)}")
        libs_in_framework = os.listdir(self.config.frameworks_path)

        added_to_framework_count = 0
        for lib in libs_found:
            if lib == self.config.embedded_python_binary:  # if it's the Python binary we continue
                continue
            base = os.path.basename(lib)
            if base not in libs_in_framework and lib not in python_libs:
                shutil.copy2(
                    lib,
                    self.config.frameworks_path,
                )  # copy libs that are not in framework yet
                added_to_framework_count = added_to_framework_count + 1
        logger.info(f"libs added to Frameworks: {added_to_framework_count}")

        logger.info(
            f" --- Python libs: set rpath to Frameworks, nb libs: {len(python_libs)}",
        )
        # TODO: remove old rpath
        dir_python_libs = self.config.embedded_python_lib.split("/")
        deep_sp = len(dir_python_libs)
        # Set the rpath to the Frameworks path
        for filename in python_libs:
            dir_fsp = filename.split("/")
            deep_lib_sp = len(dir_fsp) - deep_sp
            rpath = "@loader_path/../../../"
            for _ in range(deep_lib_sp):
                rpath += "../"
            rpath += "Frameworks"
            subprocess.run(
                ["install_name_tool", "-add_rpath", rpath, filename],
                check=False,
            )

    def _collect_dependencies(self):
        """Collect dependencies of CloudCompare binary and QT libs

        Returns
        -------
            set[str]: libs and binaries found in the collect process.
            set[str]: libs and binaries found with an @executable_path dependency.
            set[str]: libs and binaries found in the plugin dir.

        """
        # Searching for CC dependencies
        libs_to_check = []

        # results
        libs_found = set()  # Abs path of libs/binaries already checked, candidate for embedding in the bundle
        lib_ex_found = set()
        libs_in_plugins = set()

        logger.info("Adding main executable to the libs to check")
        libs_to_check.append(self.config.cc_bin_path)
        logger.info("Adding lib already available in Frameworks to the libsToCheck")
        libs_in_framework_dirs = os.listdir(self.config.frameworks_path)
        for filename in libs_in_framework_dirs:
            f = os.path.join(self.config.frameworks_path, filename)
            libs_to_check.append(f)
        logger.info(
            f"number of libs already in Frameworks directory: {len(libs_in_framework_dirs)}",
        )

        logger.info("Adding plugins to the libsToCheck")
        for plugin_dir in os.listdir(self.config.plugin_path):
            absdir = os.path.join(self.config.plugin_path, plugin_dir)
            if os.path.isdir(absdir) and ".app" not in plugin_dir:
                for filename in os.listdir(absdir):
                    f = os.path.join(absdir, filename)
                    if os.path.isfile(f) and (os.path.splitext(f)[1] in (".dylib", ".so")):
                        libs_to_check.append(f)
                        libs_in_plugins.add(f)

        logger.info(f"number of libs in PlugIns directory: {len(libs_in_plugins)}")

        logger.info("searching for dependencies...")
        while len(libs_to_check):
            # --- Unstack a binary/lib from the libs_to_check array
            lib2check = libs_to_check.pop(0)

            # If the lib was already processed we continue, of course
            if lib2check in libs_found:
                continue

            # Add the current lib to the already processed libs
            libs_found.add(lib2check)

            # search for @rpath and @executable_path dependencies in the current lib
            lib_deps, lib_ex = self._get_lib_dependencies(lib2check)

            # @executable_path are handled in a seperate set
            lib_ex_found.update(lib_ex)

            # TODO: group these two functions since we do not need
            # get all rpath for the current lib
            rpaths = CCBundler._get_rpath(lib2check)
            # get absolute path from found rpath
            abs_search_paths = CCBundler._convert_rpaths(lib2check, rpaths)

            # If the extra_pathlib is not already added, we ad it
            # TODO:: there is no way it can be False
            # maybe we should prefer to check for authorized lib_dir
            # TODO: if rpath is @loader_path, LIB is either in framework (already embedded) or in extra_pathlib
            # we can take advantage of that...
            if self.config.extra_pathlib not in abs_search_paths:
                abs_search_paths.append(self.config.extra_pathlib)

            # TODO: check if exists, else throw and exception
            for dependency in lib_deps:
                for abs_rp in abs_search_paths:
                    abslib_path = os.path.join(abs_rp, dependency)
                    if os.path.isfile(abslib_path):
                        if abslib_path not in libs_to_check and abslib_path not in libs_found:
                            # if this lib was not checked for dependencies yet, we append it to the list of lib to check
                            libs_to_check.append(abslib_path)
                        break

                # TODO; handle lib_ex
                # for dependency in lib_ex:
                abs_dep = os.path.join(self.config.cc_bin_path, dependency)
            # TODO: add to libTOcheck executable_path/dep

        return libs_found, lib_ex_found, libs_in_plugins

    def _embed_libraries(
        self,
        libs_found: set[str],
        lib_ex_found: set[str],
        libs_in_plugins: set[str],
    ) -> None:
        """Embed collected libraries into the `.app` bundle.

        rpath of embedded libs is modified to match their new location

        Args:
        ----
            libs_found (set[str]): libs and binaries found in the collect process.
            libs_ex_found (set[str]): libs and binaries found with an @executable_path dependency.
            libs_found (set[str]): libs and binaries found in the plugin dir.

        """
        logger.info("Copying libraries")
        logger.info(f"libexfound to add to Frameworks: {len(lib_ex_found)}")
        logger.info(f"libsfound to add to Frameworks: {len(libs_found)}")

        libs_in_frameworks = os.listdir(self.config.frameworks_path)

        nb_libs_added = 0
        for lib in libs_found:
            if lib == self.config.cc_bin_path:
                continue
            base = os.path.basename(lib)
            if (base not in libs_in_frameworks) and (lib not in libs_in_plugins):
                shutil.copy2(lib, self.config.frameworks_path)
                nb_libs_added += 1
        logger.info(f"number of libs added to Frameworks: {nb_libs_added}")

        # --- ajout des rpath pour les librairies du framework : framework et ccPlugins
        logger.info(" --- Frameworks libs: add rpath to Frameworks")
        nb_frameworks_libs = 0

        # TODO: purge old rpath
        for filename in os.listdir(self.config.frameworks_path):
            f = os.path.join(self.config.frameworks_path, filename)
            if os.path.isfile(f) and os.path.splitext(f)[1] in (".so", ".dylib"):
                nb_frameworks_libs += 1
                subprocess.run(
                    ["install_name_tool", "-add_rpath", "@loader_path", f],
                    stdout=subprocess.PIPE,
                    check=False,
                )
        logger.info(f"number of Frameworks libs with rpath modified: {nb_frameworks_libs}")
        logger.info(
            f" --- PlugIns libs: add rpath to Frameworks, number of libs: {len(libs_in_plugins)}",
        )
        for f in libs_in_plugins:
            if os.path.isfile(f):
                subprocess.run(
                    ["install_name_tool", "-add_rpath", "@loader_path/../../Frameworks", f],
                    stdout=subprocess.PIPE,
                    check=False,
                )

        # TODO: make a function for this
        # Embed libs with an @executable_path dependencies
        for libex in lib_ex_found:
            base = libex[0]
            target = libex[1]

            framework_path = os.path.join(self.config.frameworks_path, base)
            plugin_path = os.path.join(self.config.plugin_path + "/ccPlugins", base)

            if os.path.isfile(framework_path):
                base_path = framework_path
            elif os.path.isfile(plugin_path):
                base_path = plugin_path
            else:
                # TODO: this should not be possible
                raise Exception("no base path")

            logger.info(f"modify : @executable_path -> @rpath: {base_path}")

            subprocess.run(
                [
                    "install_name_tool",
                    "-change",
                    "@executable_path/" + target,
                    "@rpath/" + target,
                    base_path,
                ],
                stdout=subprocess.PIPE,
                check=False,
            )


if __name__ == "__main__":
    # configure logger
    formatter = " BundleCC::%(levelname)-8s:: %(message)s"
    logging.basicConfig(level=logging.INFO, format=formatter)
    std_handler = logging.StreamHandler()

    # CLI parser
    parser = argparse.ArgumentParser("CCAppBunle")
    parser.add_argument(
        "install_path",
        help="Path where the CC application is installed (CMake install dir)",
        type=Path,
    )
    # TODO: rename this extra_loader_path
    # this could be infered from the $CONDA_PATH
    parser.add_argument(
        "--extra_pathlib",
        help="Extra path to find libraries (typically current conda env lib path)",
        type=Path,
    )
    parser.add_argument(
        "--embed_python",
        help="Whether embedding python or not",
        action="store_true",
    )
    parser.add_argument(
        "--output_dependencies",
        help="Output a json files in order to debug dependency graph",
        action="store_true",
    )
    arguments = parser.parse_args()
    # convert extra_pathlib to aboslute path
    extra_pathlib = arguments.extra_pathlib.resolve()

    config = CCAppBundleConfig(
        str(arguments.install_path),
        str(extra_pathlib),
        arguments.output_dependencies,
        arguments.embed_python,
    )

    bundler = CCBundler(config)
    bundler.bundle()
