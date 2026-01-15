# Bubble Simulator for Flipper Zero

A lightweight bubble simulation app for the **Flipper Zero** 🎈

Watch bubbles rise, collide, and pop on screen. Customize the behavior of three bubble groups right from the device — adjust count, size, speed, bounciness, and pop chance.
Long-press **OK** to hide the HUD and enjoy a clean ambient animation.

## Features

* Smooth bubbling animation
* Three independently-configurable groups:

  * Small, Medium, Large
* Adjustable per group:

  * Count
  * Radius
  * Rise speed
  * Restitution (bounciness)
  * Pop chance (%)
* HUD toggle (long-press OK)
* Saves settings to `/ext/apps_data/.../bubble.cfg`
* Works with **ufbt** (no firmware patching required)

## Controls

| Button           | Action                                                   |
| ---------------- | -------------------------------------------------------- |
| **Back**         | Exit app                                                 |
| **Up / Down**    | Change which setting field is selected                   |
| **Left / Right** | Decrease / Increase value of selected setting            |
| **OK (short)**   | Cycle bubble group (Small → Medium → Large → Small)      |
| **OK (long)**    | Toggle HUD (hide/show footer + selected-group highlight) |

HUD visibility does **not** affect input — all edits still apply while the HUD is hidden.

## Build and Run (ufbt)

This app is designed to be built and launched using **ufbt**.

### Prerequisites

* Flipper Zero firmware source / environment set up
* Python with `ufbt` installed

Install ufbt (if needed):

```sh
pip install ufbt
```

### Build

From the app directory:

```sh
ufbt build
```

### Run on Flipper (USB / WiFi debug)

```sh
ufbt launch
```

This will build (if needed) and start the app on the connected Flipper.

### Install the `.fap` to SD card

You can also flash the built app to the SD card:

```sh
ufbt flash
```

or copy the resulting `.fap` manually from the `build` directory to `/ext/apps/` on your Flipper.

## Default Configuration

If no config file exists, the app starts with these defaults:

* **Small**

  * Count: 22
  * Radius: 3.0
  * Rise speed: 60.0
  * Restitution: 0.80
  * Pop chance: 100%

* **Medium**

  * Count: 10
  * Radius: 8.0
  * Rise speed: 11.0
  * Restitution: 0.15
  * Pop chance: 10%

* **Large**

  * Count: 4
  * Radius: 16.0
  * Rise speed: 4.0
  * Restitution: 0.05
  * Pop chance: 10%

User changes are saved at runtime to:

`/ext/apps_data/<appid>/bubble.cfg`

(where `<appid>` is the app’s configured ID from your `application.fam` / `App()` definition).

## Project Structure

Example layout:

* `bubble_sim_app.c` – main app source file
* `application.fam` – app metadata for ufbt / firmware
* `.gitignore` – ignores `dist` and build artifacts
* `README.md` – this file

At runtime, the app will create and update:

* `/ext/apps_data/<appid>/bubble.cfg` – persistent bubble group config

## Known Behavior / Notes

* Physics is intentionally lightweight and tuned for a “relaxing bubbles” vibe, not strict realism.
* Pop chance is stored as `0.0–1.0` internally but displayed as a percentage in the HUD.
* When the HUD is hidden, selected-group highlighting is also hidden, but group selection and edits still work.

## Contributing

Issues and pull requests are welcome.
