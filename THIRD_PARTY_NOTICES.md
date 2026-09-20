# Third-party notices and provenance

## Maker Arm reference

The comparison-only files in reference/maker-urdf were copied from the MakerMods Lab Maker Arm assets, which identify their upstream as:
https://github.com/makermods-robotics/maker-arm-sdk/tree/b30d05a/urdf/maker_arm

They are distributed under Apache License 2.0. The license at that upstream revision is preserved in reference/maker-urdf/LICENSE, and the existing reference README and revision report retain provenance. These reference files are unchanged except for the previously documented trailing newline in robot.urdf.

The Viscous runtime meshes and inertial properties were generated from the supplied Viscous STEP assembly. The new model uses CAD-derived joint axes and shorter links. The Maker reference informed rotation conventions and the simplified symmetric-slider gripper travel; these are documented as provisional in the model manifest.

## Three.js

viewer/vendor contains Three.js revision 160 and the OrbitControls and STLLoader modules. Copyright belongs to the Three.js authors. Its MIT license is preserved in viewer/vendor/LICENSE.

## Viscous sources

The collision builder uses [CoACD](https://github.com/SarahWeiii/CoACD), an MIT-licensed offline dependency, for approximate convex decomposition. See Wei, Liu, Ling and Su, “Approximate Convex Decomposition for 3D Meshes with Collision-Aware Concavity and Tree Search,” ACM Transactions on Graphics 41(4), 2022. Generated collision meshes remain derived from Gavin's supplied CAD; the CoACD library is installed separately through requirements-lock.txt.

source/2026-09-18 contains Gavin's Viscous assembly exports, supplied with permission to publish this simulation project. Source checksums and assumptions are recorded in intake.json.

No additional project-wide license is selected in this revision. Public repository visibility does not replace the licenses and attribution of third-party material.
