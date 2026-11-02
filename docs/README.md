# Docs

The vCortex site, written in [Quarkdown](https://github.com/iamgio/quarkdown) and published to
GitHub Pages by `.github/workflows/docs.yml` on every push to `main`.

## Layout

| Path | What it holds |
|---|---|
| `main.qd` | Home page and build entry point |
| `_setup.qd` | Site shell: theme, header, footer. Quarkdown adds it to every page |
| `_nav.qd` | Sidebar: one `###!` heading per section, one link per page |
| `documentation/` | One `.qd` file per page |
| `components/` | One `.qd` file per component: its function, style and script together |
| `style/` | `tokens.css` (colours, sizes) and `quarkdown.css` (the skin over Quarkdown's page) |
| `script/` | `quarkdown-dom.js`: the `VC` helpers components use to find page parts |
| `figures/` | Standalone figures linked from the docs |

Only `style/quarkdown.css` and `script/quarkdown-dom.js` name Quarkdown's generated page.
A Quarkdown upgrade should touch those two files and nothing else.

## Build

Needs Quarkdown 2.7.0, the version CI pins.

```sh
# one build into docs/_site
quarkdown c docs/main.qd --out docs --out-name _site --allow native-content --strict

# live preview while editing
quarkdown c docs/main.qd --out docs --out-name _site --allow native-content -p -w
```

## Add a page

1. Create `documentation/<name>.qd`:

   ```
   .docname {Page title}
   .include {docs}

   .tag {Reference}

   First paragraph.
   ```

2. Link it in `_nav.qd` under a section. A section heading takes a
   [Bootstrap icon](https://icons.getbootstrap.com): `###! .icon {box} Model`.

## Components

| Call | Gives |
|---|---|
| `.tag {label} variant:{good\|warn}` | Small badge; no variant is the accent colour |
| `.codefile {name}` + indented code block | Code block with a window bar and a file name |
| `.box {Title} type:{note\|tip\|warning}` + indented text | Callout (Quarkdown built-in) |
| `.mermaid` + indented diagram | Diagram, redrawn by hand by `components/sketch.qd` |

The breadcrumb, the dark-mode toggle and the header actions need no call.

To add a component, write `components/<name>.qd` and include it in `components/index.qd`.
Prefix its classes with `vc-` and use the `--vc-*` tokens from `style/tokens.css`.
