Paste this into Claude Code from the root of the unzipped repo.

---

You're picking up a hardware design session. This repo is a parametric CadQuery case for an ELP-USB16MP02-AF100 USB camera board that will mount inside the top cover of my Snapmaker U1 and look down at the bed.

Start by reading `CLAUDE.md`. It has the full context: camera specs, current design, which dimensions are assumptions, open work and my conventions. Then:

1. Set up the env (`pip install -r requirements.txt`, plus `cairosvg` for previews) and run `python3 cad/elp_16mp_case.py --out out --log-level DEBUG`. Confirm validation and the self-check pass. Render iso and section PNGs of the base and lid and look at them.
2. Give me a short status: what the design is now, the weakest margins you see, and anything in the code you'd change. Keep it to one screen.
3. Then we'll work through the design together, starting with the **tilt mount for the U1 top cover**. Before modelling it, ask me for the measurements you need: the mount location on the cover, its height above the bed, how I want to attach it, and the panel thickness. Don't guess U1 dimensions. Propose the approach (adjustable pivot vs fixed wedge vs clip) with the trade-offs, and wait for my pick.

Working rules:
- Keep everything parametric in `Params`, and add a `validate()`/`self_check()` check for each new feature.
- Both parts print without supports.
- After each change: rebuild, render and look, run `tools/build_viewer.py`, and commit with a clear message. Show me the render.
- Ask when a decision is mine. Make the call yourself on routine stuff and say what you chose.
- The GitHub repo isn't created yet. When I say it exists, add the remote and push.
