#!/usr/bin/env bash
# build.sh - macOS (AppleClang) + vcpkg setup/build script
# Usage:
#   ./build.sh                         # Debug, dynamic
#   ./build.sh --static                # Static (no dylibs, where possible)
#   ./build.sh --config Release
#   ./build.sh --clean                 # wipe build/
#   ./build.sh --verbose
#   ./build.sh --help

set -euo pipefail

# ----- CLI -----
CONFIG="Debug"
STATIC=0
CLEAN=0
VERBOSE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config|-c)
      CONFIG="${2:-}"; shift 2;;
    --static)
      STATIC=1; shift;;
    --clean)
      CLEAN=1; shift;;
    --verbose|-v)
      VERBOSE=1; shift;;
    --help|-h)
      cat <<EOF
Usage: ./build.sh [options]
  --config|-c <Debug|Release|RelWithDebInfo|MinSizeRel>   (default: Debug)
  --static                                                (use -static triplet)
  --clean                                                 (remove build dir)
  --verbose|-v
  --help|-h
EOF
      exit 0;;
    *)
      echo "Unknown option: $1"; exit 2;;
  esac
done

# ----- Colors -----
if [ -t 1 ]; then
  CYAN="\033[36m"; GREEN="\033[32m"; YELLOW="\033[33m"; RED="\033[31m"; RESET="\033[0m"
else
  CYAN=""; GREEN=""; YELLOW=""; RED=""; RESET=""
fi
Info() { echo -e "${CYAN}[INFO] $*${RESET}"; }
Ok()   { echo -e "${GREEN}[OK]   $*${RESET}"; }
Warn() { echo -e "${YELLOW}[WARN] $*${RESET}"; }
Die()  { echo -e "${RED}[ERR]  $*${RESET}"; exit 1; }

# ----- Root & sanity -----
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
[ -f "$ROOT/CMakeLists.txt" ] || Die "Run this script from your project root"

# ----- Tool checks -----
for t in git cmake; do
  command -v "$t" >/dev/null 2>&1 || Die "$t not found in PATH"
done
# Prefer Ninja if available (faster); fall back to Unix Makefiles
GEN="Unix Makefiles"
if command -v ninja >/dev/null 2>&1; then
  GEN="Ninja"
fi

# ----- Arch & triplet -----
UNAME_M="$(uname -m)"
case "$UNAME_M" in
  arm64) BASE_TRIPLET="arm64-osx" ;;
  x86_64) BASE_TRIPLET="x64-osx" ;;
  *) Die "Unsupported architecture: $UNAME_M (expected arm64 or x86_64)";;
esac
if [ $STATIC -eq 1 ]; then
  TRIPLET="${BASE_TRIPLET}-static"
else
  TRIPLET="${BASE_TRIPLET}"
fi

BUILD_DIR="$ROOT/build/${TRIPLET}-${CONFIG}"

# ----- vcpkg env -----
export VCPKG_FEATURE_FLAGS="manifests,binarycaching"
# Use a mac-friendly cache location if not set
: "${VCPKG_DEFAULT_BINARY_CACHE:=$HOME/Library/Caches/vcpkg}"
mkdir -p "$VCPKG_DEFAULT_BINARY_CACHE"

# ----- Ensure vcpkg submodule -----
if [ ! -d "$ROOT/vcpkg/.git" ]; then
  if [ -d "$ROOT/vcpkg" ]; then rm -rf "$ROOT/vcpkg"; fi
  Info "Cloning vcpkg submodule..."
  git submodule add https://github.com/microsoft/vcpkg vcpkg >/dev/null 2>&1 || true
  git submodule update --init --recursive >/dev/null 2>&1
fi
if [ ! -f "$ROOT/vcpkg/bootstrap-vcpkg.sh" ]; then
  Info "Initializing vcpkg..."
  git submodule update --init --recursive >/dev/null 2>&1
fi

# ----- Bootstrap vcpkg if missing -----
if [ ! -x "$ROOT/vcpkg/vcpkg" ]; then
  Info "Bootstrapping vcpkg (AppleClang)..."
  bash "$ROOT/vcpkg/bootstrap-vcpkg.sh"
fi
Ok "vcpkg ready"

# ----- Install deps -----
if [ -f "$ROOT/vcpkg.json" ]; then
  Info "Installing dependencies from manifest ($TRIPLET)..."
  "$ROOT/vcpkg/vcpkg" install --triplet "$TRIPLET"
else
  Warn "vcpkg.json not found - installing libpng + zlib"
  "$ROOT/vcpkg/vcpkg" install libpng zlib --triplet "$TRIPLET" --classic
fi

# ----- Clean (optional) -----
if [ $CLEAN -eq 1 ] && [ -d "$BUILD_DIR" ]; then
  Info "Cleaning $BUILD_DIR"
  rm -rf "$BUILD_DIR"
fi
mkdir -p "$BUILD_DIR"

# ----- Configure -----
TOOLCHAIN="$ROOT/vcpkg/scripts/buildsystems/vcpkg.cmake"
CFG_ARGS=(
  -S "$ROOT" -B "$BUILD_DIR"
  -G "$GEN"
  -DCMAKE_BUILD_TYPE="$CONFIG"
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN"
  -DVCPKG_TARGET_TRIPLET="$TRIPLET"
)

# For multi-arch projects you can force one with:
#   -DCMAKE_OSX_ARCHITECTURES=arm64  or  x86_64
# We default to host arch as given by the triplet.

if [ $VERBOSE -eq 1 ]; then
  Info "cmake configure args: ${CFG_ARGS[*]}"
fi

Info "Configuring CMake for AppleClang..."
cmake "${CFG_ARGS[@]}"

# ----- Build -----
Info "Building..."
if [ "$GEN" = "Ninja" ]; then
  cmake --build "$BUILD_DIR" -j
else
  # Unix Makefiles
  cmake --build "$BUILD_DIR" -- -j
fi

# ----- Locate binary -----
# Common locations (Ninja/Unix Makefiles put it in the build dir)
BIN_CANDIDATES=(
  "$BUILD_DIR/RayTracer"
  "$BUILD_DIR/src/RayTracer"
)
FOUND=""
for p in "${BIN_CANDIDATES[@]}"; do
  if [ -f "$p" ]; then FOUND="$p"; break; fi
done

if [ -z "$FOUND" ]; then
  # Fallback: search shallowly
  FOUND="$(/usr/bin/find "$BUILD_DIR" -maxdepth 2 -type f -name 'RayTracer' -perm -111 2>/dev/null | head -n1 || true)"
fi

if [ -n "$FOUND" ]; then
  Ok "Build complete -> $FOUND"
  echo "Run: \"$FOUND\""
else
  Warn "Build succeeded but 'RayTracer' not found in $BUILD_DIR"
fi