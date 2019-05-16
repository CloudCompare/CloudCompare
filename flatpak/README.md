# Flatpak

## Setup

1. Install flatpak and add the Flathub repository: https://flatpak.org/setup.
2. Install the KDE runtime and SDK: `flatpak install org.kde.Platform//5.12 org.kde.Sdk//5.12`.
3. Make sure that the submodules are checked out: `git submodule update --init --recursive`.
4. Define the build directory: `export BUILD=/tmp/build_cc_app`.


## Build & Install

To build and install the app, run:
```
flatpak-builder --user --install $BUILD flatpak/manifest.yaml
```

You can then either start the application via the desktop launchers or from command line:
```
# CloudCompare
flatpak run net.danielgm.CloudCompare
# CloudCompare Viewer
flatpak run --command=ccViewer net.danielgm.CloudCompare
```


## Manual build for development

To build the flatpak app locally without installing it, run:
```
flatpak-builder --force-clean --ccache $BUILD flatpak/manifest.yaml
```
The resulting files will be placed in the `$BUILD` folder.

You can start the flatpak app directly from the build folder via:
```
flatpak-builder --run $BUILD flatpak/manifest.yaml CloudCompare
```
