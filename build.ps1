<#  build.ps1 — full MSVC + vcpkg setup/build script
    Usage:
      ./build.ps1               # Debug, dynamic (DLLs)
      ./build.ps1 -Static       # Static (no DLLs)
      ./build.ps1 -Config Release
      ./build.ps1 -Clean        # wipe build/
#>

param(
  [ValidateSet("Debug","Release","RelWithDebInfo","MinSizeRel")]
  [string]$Config = "Debug",
  [switch]$Static,
  [switch]$Clean,
  [switch]$Verbose
)

$ErrorActionPreference = "Stop"
$PSStyle.OutputRendering = "Ansi"
function Info($m){ Write-Host "[INFO] $m" -ForegroundColor Cyan }
function Ok($m){ Write-Host "[OK]   $m" -ForegroundColor Green }
function Warn($m){ Write-Host "[WARN] $m" -ForegroundColor Yellow }
function Die($m){ Write-Host "[ERR]  $m" -ForegroundColor Red; exit 1 }

# root + sanity
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $Root
if (-not (Test-Path "$Root\CMakeLists.txt")) { Die "Run this script from your project root" }

# Triplet + build dir
$Triplet = if ($Static) { "x64-windows-static" } else { "x64-windows" }
$BuildDir = Join-Path $Root "build\$Triplet-$Config"

# ensure required tools
foreach ($t in @("git","cmake")) {
  if (-not (Get-Command $t -ErrorAction SilentlyContinue)) { Die "$t not found in PATH" }
}

# make sure binary cache exists
$env:VCPKG_FEATURE_FLAGS = "manifests,binarycaching"
if (-not $env:VCPKG_DEFAULT_BINARY_CACHE) {
  $env:VCPKG_DEFAULT_BINARY_CACHE = "$env:LOCALAPPDATA\vcpkg-cache"
}
New-Item -ItemType Directory -Force -Path $env:VCPKG_DEFAULT_BINARY_CACHE | Out-Null

# ensure valid vcpkg submodule
if (-not (Test-Path "$Root\vcpkg\.git")) {
  if (Test-Path "$Root\vcpkg") { Remove-Item -Recurse -Force "$Root\vcpkg" }
  Info "Cloning vcpkg submodule…"
  git submodule add https://github.com/microsoft/vcpkg vcpkg | Out-Null
  git submodule update --init --recursive | Out-Null
}
if (-not (Test-Path "$Root\vcpkg\bootstrap-vcpkg.bat")) {
  Info "Initializing vcpkg…"
  git submodule update --init --recursive | Out-Null
}

# bootstrap if missing
if (-not (Test-Path "$Root\vcpkg\vcpkg.exe")) {
  Info "Bootstrapping vcpkg (MSVC)…"
  & "$Root\vcpkg\bootstrap-vcpkg.bat"
  if ($LASTEXITCODE -ne 0) { Die "vcpkg bootstrap failed" }
}
Ok "vcpkg ready"

# install deps
if (Test-Path "$Root\vcpkg.json") {
  Info "Installing dependencies from manifest ($Triplet)…"
  & "$Root\vcpkg\vcpkg.exe" install --triplet $Triplet
} else {
  Warn "vcpkg.json not found — installing libpng + zlib"
  & "$Root\vcpkg\vcpkg.exe" install libpng zlib --triplet $Triplet --classic
}
if ($LASTEXITCODE -ne 0) { Die "vcpkg install failed" }

# clean if requested
if ($Clean -and (Test-Path $BuildDir)) {
  Info "Cleaning $BuildDir"
  Remove-Item -Recurse -Force $BuildDir
}

# configure CMake (MSVC only)
$Toolchain = "$Root\vcpkg\scripts\buildsystems\vcpkg.cmake"
$cfg = @(
  "-S",$Root,"-B",$BuildDir,
  "-G","Visual Studio 17 2022",
  "-A","x64",
  "-DCMAKE_TOOLCHAIN_FILE=$Toolchain",
  "-DVCPKG_TARGET_TRIPLET=$Triplet"
)
if ($Static) {
  $crt = ("MultiThreaded" + $(if ($Config -eq "Debug") { "Debug" } else { "" }))
  $cfg += "-DCMAKE_MSVC_RUNTIME_LIBRARY=$crt"
}
if ($Verbose) { $cfg += @("-DCMAKE_FIND_DEBUG_MODE=ON","-DCMAKE_MESSAGE_LOG_LEVEL=VERBOSE") }

Info "Configuring CMake for MSVC..."
cmake @cfg
if ($LASTEXITCODE -ne 0) { Die "CMake configure failed" }

# build
Info "Building..."
cmake --build $BuildDir --config $Config -- /m
if ($LASTEXITCODE -ne 0) { Die "Build failed" }

# locate exe
$exe = Get-ChildItem "$BuildDir\$Config\RayTracer.exe" -ErrorAction SilentlyContinue
if ($exe) {
  Ok "Build complete → $($exe.FullName)"
  Write-Host "Run: `"$($exe.FullName)`""
} else {
  Warn "Build succeeded but RayTracer.exe not found in $BuildDir\$Config"
}
