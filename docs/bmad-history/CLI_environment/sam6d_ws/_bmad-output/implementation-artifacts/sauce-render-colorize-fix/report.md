# Sauce Template Colorize Fix Report

Date: 2026-06-12

## Summary

`sam6d_master/SAM-6D/Render/render_custom_templates.py` parsed `--colorize False` as the non-empty string `"False"`, which is truthy in Python. This enabled the uniform material override and produced grayscale-like sauce template RGB images. The option now uses `action='store_true'`.

Chosen behavior:

- Omit `--colorize`: keep CAD vertex colors/textures.
- Use `--colorize`: override CAD material with a uniform base color.
- `--colorize False`: rejected by argparse instead of being treated as true.

## Backup And Diff

- Backup: `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/render_custom_templates.py.before`
- Diff: `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/render_custom_templates.py.diff`
- Existing output listing before rerender: `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/existing_template_sauce_files_before.txt`
- Before stats: `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/before_grayscale_stats.json`
- After stats: `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/after_color_stats.json`

## Code Change

```diff
-parser.add_argument('--colorize', default=False, help="Whether to colorize CAD model or not")
+parser.add_argument(
+    '--colorize',
+    action='store_true',
+    help="Override the CAD material with a uniform base color. Omit to keep vertex colors/textures.",
+)
```

## Commands

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sam6d_ros_humble
export PYTHONPATH=/home/etri/miniconda3/envs/sam6d_test/lib/python3.11/site-packages${PYTHONPATH:+:$PYTHONPATH}

python /home/etri/miniconda3/envs/sam6d_test/bin/blenderproc run \
  sam6d_master/SAM-6D/Render/render_custom_templates.py \
  --cad_path data/cad/sauce/vertex_160000/Sauce_with_normal_vertexcolor.ply \
  --output_dir template/sauce/vertex_160000/Sauce_with_normal_vertexcolor \
  --normalize True

python /home/etri/miniconda3/envs/sam6d_test/bin/blenderproc run \
  sam6d_master/SAM-6D/Render/render_custom_templates.py \
  --cad_path data/cad/sauce/vertex_630000/Sauce_with_normal_vertexcolor.ply \
  --output_dir template/sauce/vertex_630000/Sauce_with_normal_vertexcolor \
  --normalize True
```

`--colorize` was intentionally omitted.

## Existing Result Handling

Both existing output folders existed before rerender:

- `template/sauce/vertex_160000/Sauce_with_normal_vertexcolor/templates`
- `template/sauce/vertex_630000/Sauce_with_normal_vertexcolor/templates`

Each had 42 `rgb_*.png`, 42 `mask_*.png`, and 42 `xyz_*.npy` files. The renderer writes deterministic file names with `cv2.imwrite` and `np.save`, so the folders were kept and overwritten in place. No deletion was performed.

## Results

| Input PLY | Output Folder | Generated RGB Images | Unique Colors | RGB Mean | RGB Std | Status |
| --- | --- | ---: | ---: | --- | --- | --- |
| `data/cad/sauce/vertex_160000/Sauce_with_normal_vertexcolor.ply` | `template/sauce/vertex_160000/Sauce_with_normal_vertexcolor/templates` | 42 | 127917 | [149.53, 129.82, 111.38] | [55.67, 55.44, 69.32] | Success |
| `data/cad/sauce/vertex_630000/Sauce_with_normal_vertexcolor.ply` | `template/sauce/vertex_630000/Sauce_with_normal_vertexcolor/templates` | 42 | 112423 | [149.48, 129.71, 111.24] | [55.67, 55.31, 69.17] | Success |

## Comparison

| Target | Unique Colors | RGB Mean | RGB Std | Verdict |
| --- | ---: | --- | --- | --- |
| Milk reference | 30075 | [216.60, 241.81, 249.29] | [75.59, 23.63, 15.37] | Color reference |
| Sauce 160k before | 601 | [72.23, 72.28, 72.23] | [18.48, 18.47, 18.47] | Grayscale-like |
| Sauce 630k before | 599 | [72.23, 72.28, 72.23] | [18.48, 18.47, 18.47] | Grayscale-like |
| Sauce 160k after | 127917 | [149.53, 129.82, 111.38] | [55.67, 55.44, 69.32] | Vertex color reflected |
| Sauce 630k after | 112423 | [149.48, 129.71, 111.24] | [55.67, 55.31, 69.17] | Vertex color reflected |

## Contact Sheet

- `_bmad-output/implementation-artifacts/sauce-render-colorize-fix/sauce_colorized_contact_sheet.png`

## Verification

- `python -m py_compile sam6d_master/SAM-6D/Render/render_custom_templates.py`: pass
- `git diff --check`: pass
- Argparse behavior:
  - omitted `--colorize`: `False`
  - `--colorize`: `True`
  - `--colorize False`: rejected
- PNG/NPY verification:
  - both sauce outputs have 42 RGB, 42 mask, 42 xyz files
  - no bad PNGs
  - no empty masks
  - no bad xyz arrays

## Conclusion

The previous grayscale-like sauce templates were caused by string truthiness from `--colorize False`. After switching to `action='store_true'` and rerendering without `--colorize`, vertex color is reflected in both sauce template sets. No additional renderer change is required for vertex-color-based templates.
