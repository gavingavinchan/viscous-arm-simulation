# Large twin-screw ship stern visual references

The scene is a reference-informed generic large ship stern, not a reconstruction of a named vessel. Metres are used throughout. Two 4.6 m diameter, five-bladed bronze propellers are 7.6 m apart; the continuous aft hull is about 25 m across. The vessel extends forward 60 m beyond the stern asset. The user-selected arrangement has **two propellers and one centreline rudder** aft of the central skeg. Both screws are unobstructed from astern. Cleaning takes place underwater with the propellers stopped.

## Images inspected

- [Genova Industrie Navali — Maintenance works on the NCL Epic](https://www.ginholding.com/en/2020-12-08/maintenance-works-on-the-ncl-epic), shipyard's own image. [Full image](https://www.ginholding.com/photo-news/281/L_HD_2020_1110_NCL%20EPIC_A_5330.jpg). Downloaded and visually inspected 2026-09-20. The hull rises over two widely separated bronze screws; a substantial centreline skeg divides the shaft passages, full-height rudder foils sit aft of the screws, and bracket structures connect the exposed shafts to the hull. The human-scale scaffolding shows that a compact inspection robot should occupy only a fraction of one blade. Worn red antifouling, welded plate lines, darker patches and exposed bronze informed the materials. Dry-dock support blocks/scaffolding are intentionally absent in the underwater asset.
- [Anschütz — autopilots on twin-rudder vessels](https://www.anschuetz.com/news/article/anschuetz-autopilots-save-up-to-5-fuel-on-twin-rudder-vessels). [Manufacturer's image](https://www.anschuetz.com/fileadmin/_processed_/7/1/csm_autopilots_01_4abad597b6.jpg), found via image search. Provides reference for the twin screws and broad overhead hull. Its paired rudders are not copied: the user explicitly requested a single centreline rudder.
- [Wärtsilä — customised controllable pitch propeller systems](https://www.wartsila.com/marine/products/propulsors-and-gears/propellers/wartsila-controllable-pitch-propeller-systems): primary manufacturer context for large twin-screw cargo/RoRo applications.
- [Wärtsilä marine encyclopedia — skeg](https://www.wartsila.com/encyclopedia/term/skeg): reference for the finlike projection beneath the stern.

The downloadable reference photograph was used only to guide original procedural geometry/materials; it is not embedded in the website film. Blade shape, hull proportions and supports are cinematic approximations, not engineering drawings. The photographs show paired rudders; this asset intentionally follows the user-selected twin-propeller/single-centre-rudder layout instead of reproducing those photos exactly.

## Import interface

`scripts/ship_stern_asset.py: build_ship_stern(origin=(0,0,0), prop_diameter=4.6, prop_spacing=7.6, barnacles=True)`.

Coordinates: Z up, +Y toward bow, aft faces point roughly -Y. Propeller centres are (-3.8, 0, 0) and (3.8, 0, 0). The hull underside rises above z=2.8 at the propeller plane. One centreline rudder is at x=0, aft of the central skeg at negative Y. There are no rudders immediately behind either propeller.

Returns a dictionary with `collection`, `hull`, `rudder`, `propellers`, ten `blades`, `materials`, `prop_centres`, `prop_radius`, `cleaning_point`, `cleaning_normal`, `cleaning_blade`, `cleaning_parameters`, and `blade_surface(pi, bi, u, v)` returning world point and aft-facing normal. `u` is radial root-to-tip and `v` crosses blade chord. The default work point is on the port propeller's outer/lower blade, clear of the rudder; this work blade has clean bronze material and no static shell geometry so the shot builder can add removal animation.

`cinematic/ship_stern_work_patch.json` contains world-space edge anchor, pad position, normal, tangent, radial axis and corresponding parameters. The pad is 0.19 m in from the boundary measured along the nominal chord. The exposed blade thickness at the anchor parameter is about 7 mm, suitable for a gripper closing around the edge. Pure module-level `blade_surface(...)` and `blade_width(...)` queries can run before scene construction.
