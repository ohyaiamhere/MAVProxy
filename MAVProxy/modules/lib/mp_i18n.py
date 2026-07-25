#!/usr/bin/env python3
'''
Internationalisation support for MAVProxy user-facing strings.

User-visible text (console messages, status lines, errors, help
descriptions, etc.) is stored in language catalog files under
MAVProxy/lang/. Each language file defines a STRINGS dictionary that
maps stable message keys to translated text.

CLI command names themselves remain English; only text that MAVProxy
displays to the user is translated.

Usage:
    from MAVProxy.modules.lib.mp_i18n import tr, set_language, get_language

    print(tr("loaded_module", modname))
    print(tr("usage_module"))

Language can be selected with --language on the command line, or at
runtime with: set language de

AP_FLAKE8_CLEAN
'''

import os
import sys
import importlib

# Default language
_DEFAULT_LANGUAGE = 'en'

# Currently active language code (e.g. 'en', 'de', 'fr')
_language = _DEFAULT_LANGUAGE

# Active string table (key -> text)
_strings = {}

# English fallback table
_english = {}

# Reverse map: English source text -> message key (for late re-translation of
# strings that were resolved with tr() at module-load time)
_english_to_key = {}

# Optional callback when language changes (e.g. to refresh UI)
_on_language_change = None

# Directory containing language modules
_LANG_PACKAGE = 'MAVProxy.lang'


def _load_catalog(lang):
    '''Load STRINGS dict for a language code. Returns dict or None.'''
    try:
        mod = importlib.import_module('%s.%s' % (_LANG_PACKAGE, lang))
        # reload so runtime language file edits are picked up if desired
        importlib.reload(mod)
        strings = getattr(mod, 'STRINGS', None)
        if isinstance(strings, dict):
            return strings
    except ImportError:
        return None
    except Exception as e:
        print("mp_i18n: failed to load language '%s': %s" % (lang, e))
        return None
    return None


def _rebuild_english_reverse_map():
    '''Build English text -> key map (first key wins on duplicate texts).'''
    global _english_to_key
    rev = {}
    for key, text in _english.items():
        # Prefer keeping the first registration; duplicates are rare
        if text not in rev:
            rev[text] = key
    _english_to_key = rev


def init(language=None):
    '''Initialise i18n, optionally selecting a language.

    Safe to call multiple times. Loads English as the fallback, then
    the requested language on top.

    Returns True if the requested language catalog was found (or English
    was requested), False if we fell back to English because the catalog
    was missing.
    '''
    global _english, _strings, _language

    eng = _load_catalog('en')
    if eng is None:
        eng = {}
    _english = eng
    _rebuild_english_reverse_map()

    if language is None:
        language = _DEFAULT_LANGUAGE

    return set_language(language)


def set_language(language):
    '''Switch the active language.

    Falls back to English for any missing keys. Returns True if the
    language catalog was found, False if we fell back to English only.
    '''
    global _strings, _language, _english

    if not _english:
        eng = _load_catalog('en')
        _english = eng if eng is not None else {}
        _rebuild_english_reverse_map()

    language = (language or _DEFAULT_LANGUAGE).lower().strip()
    # normalise common aliases
    aliases = {
        'english': 'en',
        'german': 'de',
        'deutsch': 'de',
        'french': 'fr',
        'francais': 'fr',
        'français': 'fr',
        'spanish': 'es',
        'espanol': 'es',
        'español': 'es',
        'italian': 'it',
        'italiano': 'it',
        'hindi': 'hi',
        'portuguese': 'pt',
        'portugues': 'pt',
        'português': 'pt',
        'tamil': 'ta',
        'telugu': 'te',
    }
    language = aliases.get(language, language)

    found = True
    if language == 'en':
        _strings = dict(_english)
    else:
        catalog = _load_catalog(language)
        if catalog is None:
            found = False
            _strings = dict(_english)
            language = 'en'
        else:
            # English as base, overlay translations
            _strings = dict(_english)
            _strings.update(catalog)

    _language = language

    if _on_language_change is not None:
        try:
            _on_language_change(language)
        except Exception:
            pass

    return found


def get_language():
    '''Return the currently active language code.'''
    return _language


def available_languages():
    '''Return a sorted list of available language codes.'''
    langs = set()
    try:
        import MAVProxy.lang as langpkg
        pkg_dir = os.path.dirname(langpkg.__file__)
    except Exception:
        # fall back to path relative to this file
        pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'lang')
        pkg_dir = os.path.normpath(pkg_dir)

    if os.path.isdir(pkg_dir):
        for name in os.listdir(pkg_dir):
            if name.endswith('.py') and not name.startswith('_'):
                langs.add(name[:-3])
    if not langs:
        langs.add('en')
    return sorted(langs)


def set_language_change_callback(callback):
    '''Register a callback invoked as callback(lang) after set_language.'''
    global _on_language_change
    _on_language_change = callback


def resolve_key(key_or_text):
    '''Resolve a message key, accepting a key or already-translated text.

    Modules often call tr("some_key") at import/load time and store the
    resulting string (English or another language). resolve_key maps that
    back to the stable message key so tr() can re-translate after
    `set language`.
    '''
    if not isinstance(key_or_text, str):
        return key_or_text
    if not _strings and not _english:
        init()
    # Already a catalog key?
    if key_or_text in _english:
        return key_or_text
    # English source text previously returned by tr()?
    if key_or_text in _english_to_key:
        return _english_to_key[key_or_text]
    # Text from the active (or partial) catalog?
    for k, v in _strings.items():
        if v == key_or_text:
            return k
    return key_or_text


def get(key, default=None):
    '''Return the raw translated string for key (no formatting).

    `key` may be a stable message key or the original English text.
    Falls back to English, then to default, then to the key itself.
    '''
    if not _strings and not _english:
        init()
    key = resolve_key(key)
    if key in _strings:
        return _strings[key]
    if key in _english:
        return _english[key]
    if default is not None:
        return default
    return key


def t(key, *args, **kwargs):
    '''Translate a message key and optionally format it.

    Examples:
        t("flight_logs_full")
        t("loaded_module", modname)                 # printf-style %
        t("invalid_json_argument", arg=x, err=e)    # str.format kwargs
        t("s_s", name, desc)                        # multiple % args

    `key` may also be English source text previously returned by tr(), so
    that runtime language switches re-translate early-bound strings.

    If positional args are given, the translation is formatted with the
    % operator. If keyword args are given (and no positional args),
    str.format is used instead.
    '''
    text = get(key)
    if args:
        try:
            return text % args
        except Exception:
            try:
                # single non-tuple arg often used with %s
                if len(args) == 1:
                    return text % args[0]
            except Exception:
                pass
            return text
    if kwargs:
        try:
            return text.format(**kwargs)
        except Exception:
            return text
    return text


# Common alias (gettext-style)
_ = t
tr = t  # preferred name (avoids shadowing local "t")


def ngettext(singular_key, plural_key, n, *args):
    '''Choose singular or plural key based on n, then translate.

    Simple English-style pluralisation (n == 1 -> singular). Language
    catalogs can still supply appropriate wording for each key.
    '''
    key = singular_key if n == 1 else plural_key
    if args:
        return t(key, *args)
    return t(key)


def early_language_from_argv(argv=None):
    '''Read --language from argv (or MAVPROXY_LANGUAGE) before parsers run.

    Needed so help=tr(...) and --help output use the requested language.
    '''
    if argv is None:
        argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == '--language' and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith('--language='):
            return a.split('=', 1)[1]
    env = os.environ.get('MAVPROXY_LANGUAGE')
    if env:
        return env
    return _DEFAULT_LANGUAGE


def ensure_language_from_argv(argv=None):
    '''Initialise i18n from --language / env. Returns active language code.'''
    lang = early_language_from_argv(argv)
    init(lang)
    return get_language()


def I18nOptionParser(*args, **kwargs):
    '''optparse.OptionParser with translated Usage/Options headings and -h text.

    Call ensure_language_from_argv() before constructing the parser so that
    help=tr(...) strings are evaluated in the correct language.
    '''
    from optparse import OptionParser

    class _I18nOptionParser(OptionParser):
        def __init__(self, *a, **kw):
            kw = dict(kw)
            kw['add_help_option'] = False
            OptionParser.__init__(self, *a, **kw)
            self.add_option(
                "-h", "--help", action="help",
                help=tr("opt_show_this_help_message_and_exit"))

        def format_help(self, formatter=None):
            text = OptionParser.format_help(self, formatter)
            text = text.replace("Usage:", tr("opt_heading_usage") + ":", 1)
            text = text.replace("Options:", tr("opt_heading_options") + ":", 1)
            return text

    return _I18nOptionParser(*args, **kwargs)


# Eagerly load English so early prints work before main() calls init()
try:
    init('en')
except Exception:
    pass
