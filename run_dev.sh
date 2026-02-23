#!/bin/bash
# Development script to run CloudCompare with proper paths set

BUILD_DIR="$(cd "$(dirname "$0")/build" && pwd)"
PLUGIN_DIRS=$(find "$BUILD_DIR/plugins/core" -name "*.so" -exec dirname {} \; | sort -u | tr '\n' ':' | sed 's/:$//')
EXAMPLE_PLUGIN_DIRS=$(find "$BUILD_DIR/plugins/example" -name "*.so" -exec dirname {} \; 2>/dev/null | sort -u | tr '\n' ':' | sed 's/:$//')
if [ -n "$EXAMPLE_PLUGIN_DIRS" ]; then
    PLUGIN_DIRS="$PLUGIN_DIRS:$EXAMPLE_PLUGIN_DIRS"
fi
export CC_PLUGIN_PATH="$PLUGIN_DIRS"
export CC_SHADER_PATH="$BUILD_DIR/shaders"

echo "Running CloudCompare with:"
echo "  Plugins: $CC_PLUGIN_PATH"
echo "  Shaders: $CC_SHADER_PATH"
echo ""

exec "$BUILD_DIR/qCC/CloudCompare" "$@"
