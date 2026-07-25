# MAVProxy language catalogs

User-facing text (console messages, errors, status lines, help
descriptions, and similar) is stored in per-language Python modules in
this directory. CLI **command names** stay in English; only text that
MAVProxy *displays* to the user is translated.

## Layout

| File   | Language |
|--------|----------|
| `en.py` | English (source of truth — every message key lives here) |
| `de.py` | German |
| `fr.py` | French |
| `es.py` | Spanish |
| `it.py` | Italian |
| `hi.py` | Hindi (Devanagari) |
| `pt.py` | Portuguese |
| `ta.py` | Tamil |
| `te.py` | Telugu |

Each file defines a `STRINGS` dictionary:

```python
STRINGS = {
    'loaded_module': 'Loaded module %s',
    'no_link': 'no link',
    ...
}
```

Keys are stable identifiers. Values may use printf-style placeholders
(`%s`, `%d`, `%f`, …) or `str.format` placeholders (`{0}`, `{name}`).

Prefer stable key names that do **not** embed a specific language list
(e.g. use `set_ui_language_code`, not `set_ui_language_code_en_de_fr`).

## How lookup works

```python
from MAVProxy.modules.lib.mp_i18n import tr

print(tr('loaded_module', modname))   # format with %
print(tr('loaded_module') % modname)  # equivalent
print(tr('no_link'))
```

1. Look up the key in the active language catalog.
2. If missing, fall back to English (`en.py`).
3. If still missing, return the key itself (so unknown keys are visible).

## Selecting a language

**Command line:**

```bash
mavproxy.py --language de
mavproxy.py --language fr
mavproxy.py --language es
mavproxy.py --language it
mavproxy.py --language hi
mavproxy.py --language pt
mavproxy.py --language ta
mavproxy.py --language te
```

**At runtime:**

```
set language de
set language pt
set language ta
set language en
```

Available codes are the basenames of the modules in this directory
(currently `de`, `en`, `es`, `fr`, `hi`, `it`, `pt`, `ta`, `te`).

Aliases: `german`/`deutsch`, `french`/`francais`, `spanish`/`espanol`,
`italian`/`italiano`, `hindi`, `portuguese`/`portugues`, `tamil`,
`telugu`, `english`.

## Adding a new language

1. Create `MAVProxy/lang/<code>.py` (e.g. `nl.py` for Dutch).
2. Define a `STRINGS` dict covering the same keys as `en.py` (or as many
   as you can; missing keys fall back to English).
3. No code changes are required for discovery — `available_languages()`
   scans this directory.
4. Add the code to `REQUIRED_LANGS` in `MAVProxy/lang/tests/test_i18n.py`.
5. Run the i18n autotests (below) and fix any failures.

Minimal example:

```python
"""Portuguese language strings for MAVProxy."""

STRINGS = {
    'loaded_module': 'Módulo %s carregado',
    'no_link': 'sem ligação',
    'unknown_command': "Comando desconhecido '%s'",
}
```

## Adding a new user-facing string

1. Add a key/value to `en.py` (use a stable key name; do not encode a
   temporary language list in the key).
2. Use `tr('your_key')` (and optional format args) at the call site.
3. Optionally add translations for the same key in the other language files.
4. Run the i18n autotests.

Prefer clear, stable keys (`module_not_loaded`) over numbering.

## Scope notes

- **Translated:** messages the user reads (prints, console, `say()`,
  usage/help text, errors, status).
- **Not translated:** CLI command tokens (`module`, `load`, `set`, …),
  MAVLink field names, parameter names, log file formats, and other
  machine-oriented identifiers.

## Command-line `--help`

`--help` text is translated. Language is read from argv **before** the
option parser is built, so both of these work:

```bash
mavproxy.py --language de --help
mavproxy.py --help --language de
```

You can also set the environment variable `MAVPROXY_LANGUAGE` (e.g. `de`).

## Technical terms and short forms

Do **not** machine-translate aviation/GCS short forms as ordinary words.
Keep command tokens in usage lines in English. Preferred wording:

| Term | Meaning | Notes |
|------|---------|--------|
| magcal | magnetometer calibration | keep token `magcal` in text |
| magic (0x…) | file/protocol magic number | not “wizardry” |
| magical | MAVProxy module name | keep `magical` |
| fence | geofence | not garden fence |
| rally | rally point | ArduPilot term |
| arm / disarm | motor arming | keep arm/disarm tokens |
| guided | GUIDED flight mode | keep GUIDED |
| forcecal | force calibration | keep `forcecal` |

When adding translations, prefer leaving CLI tokens (`magcal`, `fence load`, …)
unchanged so command syntax stays usable.

## Autotests

Automated tests live in `MAVProxy/lang/tests/`. They check that:

- All required language catalogs import and are non-empty
- Non-English catalogs include every English key
- Language switching and aliases work
- `tr()` formatting and unknown-key fallback behave correctly
- Early-bound English strings re-translate after `set language`
- Technical short forms such as `magcal` are not mistranslated as “magic”
- Language UI message keys use stable names (not `*_en_de_fr` suffixes)
- `I18nOptionParser` produces translated help headings

Run them from the **repository root** (`MAVProxy` checkout, the directory
that contains the top-level `MAVProxy/` package and `setup.py`).

### Run all i18n tests (quiet)

```bash
cd /path/to/MAVProxy && PYTHONPATH=. python3 -m unittest discover -s MAVProxy/lang/tests -q
```

### Run all i18n tests (verbose)

```bash
cd /path/to/MAVProxy && PYTHONPATH=. python3 -m unittest discover -s MAVProxy/lang/tests -v
```

### Equivalent one-liners if you are already at the repo root

Quiet:

```bash
PYTHONPATH=. python3 -m unittest discover -s MAVProxy/lang/tests -q
```

Verbose:

```bash
PYTHONPATH=. python3 -m unittest discover -s MAVProxy/lang/tests -v
```

### Run a single test module as a script

Quiet:

```bash
PYTHONPATH=. python3 MAVProxy/lang/tests/test_i18n.py -q
```

Verbose:

```bash
PYTHONPATH=. python3 MAVProxy/lang/tests/test_i18n.py -v
```

A successful run ends with `OK` and exit code `0`. Fix any failures
before merging catalog or i18n-related changes.
