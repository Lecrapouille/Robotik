# 🛠 Documentation tools

[⏫ Documentation](../README.md)

The diagrams of the documentation are SVG files, next to the pages that show them (`doc/<folder>/images/`). Most of them are drawn by [diagrams.py](diagrams.py), with the boxes, arrows and colours of [svgkit.py](svgkit.py). The same colours have the same meaning everywhere:

| Colour | Meaning |
|--------|---------|
| orange | the code that you write (plugin, skill, scenario logic) |
| green | the files that you write (YAML, PDDL) |
| blue | RobotIK |
| grey | a third-party library |
| dashed | planned, not done yet |

To change a diagram, edit its function in `diagrams.py`, then run:

```bash
python3 doc/tools/diagrams.py
magick doc/concepts/images/layers.svg /tmp/layers.png   # look at the result
```

The script needs Python 3 only. It writes every diagram again: compare the result with `git diff`. A few drawings are written by hand (the docstring of `diagrams.py` lists them): edit those SVG files directly, in the same style.

Do not write a diagram in ASCII art in a page: draw it here.
