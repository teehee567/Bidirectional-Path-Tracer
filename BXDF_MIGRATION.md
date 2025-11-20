# BxDF Material System Migration

## Summary

This document describes the migration from the old material system to the new BxDF-based material system.

## Changes Made

### 1. Scene Loader Updates (`src/scene/scene_loader.h`)

Added support for loading BxDF-based materials from YAML scene files. The scene loader now recognizes the following new material types:

#### New BxDF Material Types

- **`lambertian_bxdf`**: Lambertian diffuse reflection
  - Parameters: `color`/`albedo`
  - Example: `type: lambertian_bxdf, color: [255, 97, 3]`

- **`specular_reflection`**: Perfect mirror reflection
  - Parameters: `color` (reflectance)
  - Example: `type: specular_reflection, color: [1.0, 1.0, 1.0]`

- **`specular_transmission`**: Perfect transmission (glass without roughness)
  - Parameters: `eta_a`, `eta_b`, `color` (or `ior` as shorthand for `eta_b`)
  - Example: `type: specular_transmission, eta_a: 1.0, eta_b: 1.5`

- **`biggx`**: Bidirectional GGX (handles both reflection and transmission with microfacets)
  - Parameters: `ior`, `roughness`
  - Example: `type: biggx, ior: 1.5, roughness: 0.001`
  - Best for: Glass, water, and other dielectrics with realistic roughness

- **`ggx`**: GGX reflection only (no transmission)
  - Parameters: `f0` (Fresnel reflectance), `roughness`
  - Example: `type: ggx, f0: [147, 147, 147], roughness: 0.1`
  - Best for: Metals and rough surfaces

- **`mtl_ggx`**: Material GGX with separate diffuse/specular
  - Parameters: `diffuse`, `specular`, `roughness`, `ior`, `dissolve`
  - Example: `type: mtl_ggx, diffuse: [200, 100, 50], specular: [255, 255, 255], roughness: 0.2, ior: 1.5, dissolve: 1.0`

- **`mtl_ext_ggx`**: Extended Material GGX with metallic workflow
  - Parameters: `base_color`, `metallic`, `roughness`, `ior`, `dissolve`
  - Example: `type: mtl_ext_ggx, base_color: [200, 100, 50], metallic: 0.8, roughness: 0.2, ior: 1.5, dissolve: 1.0`

### 2. Glass of Water Scene Migration (`scenes/glass-of-water.yaml`)

The glass-of-water scene has been migrated to use the new BxDF materials:

#### Material Conversions

| Original Material | Type       | Original Params              | New Material | New Type | New Params                    |
|-------------------|------------|------------------------------|--------------|----------|-------------------------------|
| Backdrop          | metal      | color: [147,147,147], r: 0.1 | Backdrop     | ggx      | f0: [147,147,147], r: 0.1     |
| Floor             | metal      | color: [147,147,147], r: 0.1 | Floor        | ggx      | f0: [147,147,147], r: 0.1     |
| WaterAir          | dielectric | ior: 1.33                    | WaterAir     | biggx    | ior: 1.33, r: 0.001           |
| IceAir            | dielectric | ior: 1.31                    | IceAir       | biggx    | ior: 1.31, r: 0.005           |
| Glass             | dielectric | ior: 1.5                     | Glass        | biggx    | ior: 1.5, r: 0.001            |
| AirIce            | dielectric | ior: 0.763                   | AirIce       | biggx    | ior: 1.31, r: 0.005           |
| Light             | light      | emission: [...]              | Light        | light    | emission: [...] (unchanged)   |

#### Key Improvements

1. **Metal surfaces (Backdrop, Floor)**:
   - Migrated from simple `metal` to `ggx` for more physically accurate reflection
   - Uses `f0` (Fresnel reflectance at normal incidence) instead of `color`
   - Maintains the same roughness value (0.1)

2. **Dielectric surfaces (Water, Ice, Glass)**:
   - Migrated from simple `dielectric` to `biggx` for physically-based rendering
   - BiGGX handles both reflection and transmission with microfacet theory
   - Added small roughness values for more realistic surfaces:
     - Water: 0.001 (very smooth)
     - Glass: 0.001 (very smooth)
     - Ice: 0.005 (slightly rough)

3. **AirIce interface**:
   - Original used a ratio IOR of 0.763 (which is 1/1.31)
   - Converted to use absolute IOR of 1.31 (ice)
   - BiGGX automatically handles the bidirectional nature of the interface

## Benefits of the New System

1. **More Physically Accurate**: BxDF materials are based on microfacet theory and provide more realistic light interaction
2. **Better Caustics**: The BiGGX material will produce better caustics effects in the glass-of-water scene
3. **Unified Framework**: All materials now use a consistent BxDF interface
4. **Future Extensibility**: Easy to add new BxDF types without modifying the core material system

## Backward Compatibility

The old material types (`lambertian`, `metal`, `dielectric`, `light`) are still supported in the scene loader for backward compatibility. However, new scenes should use the BxDF material types for better rendering quality.

## Testing

To test the changes:

```bash
cd /Users/hi/Documents/Github/Bidirectional-Path-Tracer
./build.sh
./build/arm64-osx-Release/RayTracer scenes/glass-of-water.yaml
```

The rendered image will be saved to `glass_of_water.png` in the output directory.

