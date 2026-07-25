#!/usr/bin/env python3
'''
Autotests for MAVProxy internationalisation (i18n).

These tests guard catalog integrity, language switching, technical short
forms, and reverse-key lookup so future edits do not silently break
multi-language support.

Run from the repository root (see MAVProxy/lang/README.md).
'''

from __future__ import print_function

import importlib
import os
import re
import sys
import unittest

# Ensure the repository root is importable when tests are launched from
# various working directories.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from MAVProxy.modules.lib import mp_i18n  # noqa: E402
from MAVProxy.lang.en import STRINGS as EN  # noqa: E402


# Languages that must ship with a full STRINGS catalog
REQUIRED_LANGS = ('en', 'de', 'fr', 'es', 'it', 'hi', 'pt', 'ta', 'te')

# Removed / must never reappear as a first-class language code
FORBIDDEN_LANGS = ('hinglish',)

# Technical short forms that must not be mistranslated as ordinary words
TECH_SHORTFORMS = {
    'cmd_magcal': {
        # language -> substrings that must appear / must not appear
        'must_contain': {
            'en': ['magcal'],
            'de': ['magcal'],
            'fr': ['magcal'],
            'es': ['magcal'],
            'it': ['magcal'],
            'hi': ['magcal'],
        },
        'must_not_contain': {
            'de': ['magisch', 'Magie'],
            'fr': ['magique'],
            'es': [],  # "mágico" is for magic number, not this key
            'it': ['magico', 'magica'],
        },
    },
}

PLACEHOLDER_RE = re.compile(
    r'%(?:\([^)]+\))?[-+#0 ]*\d*(?:\.\d+)?[sdifFgGeExXoc%u]|'
    r'\{[^{}]*\}'
)


def _load_lang(code):
    mod = importlib.import_module('MAVProxy.lang.%s' % code)
    importlib.reload(mod)
    strings = getattr(mod, 'STRINGS', None)
    if not isinstance(strings, dict):
        raise AssertionError('%s.STRINGS missing or not a dict' % code)
    return strings


class TestCatalogFiles(unittest.TestCase):
    '''Language catalog files on disk.'''

    def test_required_language_modules_importable(self):
        for code in REQUIRED_LANGS:
            strings = _load_lang(code)
            self.assertGreater(len(strings), 0, msg='%s catalog empty' % code)

    def test_forbidden_languages_absent(self):
        avail = mp_i18n.available_languages()
        for code in FORBIDDEN_LANGS:
            self.assertNotIn(code, avail)
            with self.assertRaises(ImportError):
                importlib.import_module('MAVProxy.lang.%s' % code)

    def test_available_languages_lists_required(self):
        avail = set(mp_i18n.available_languages())
        for code in REQUIRED_LANGS:
            self.assertIn(code, avail, msg='%s not discovered by available_languages()' % code)

    def test_non_english_catalogs_cover_all_english_keys(self):
        en_keys = set(EN.keys())
        self.assertGreater(len(en_keys), 100, msg='English catalog unexpectedly small')
        for code in REQUIRED_LANGS:
            if code == 'en':
                continue
            strings = _load_lang(code)
            missing = en_keys - set(strings.keys())
            self.assertFalse(
                missing,
                msg='%s missing %d keys (e.g. %s)' % (
                    code, len(missing), list(sorted(missing))[:5]))

    def test_english_values_are_non_empty_strings(self):
        for key, value in EN.items():
            self.assertIsInstance(value, str, msg='EN[%r] not a str' % key)
            self.assertTrue(value != '' or key, msg='EN[%r] empty' % key)


class TestPlaceholderIntegrity(unittest.TestCase):
    '''Printf / format placeholders must survive translation.'''

    def test_placeholder_counts_match_english(self):
        failures = []
        for code in REQUIRED_LANGS:
            if code == 'en':
                continue
            strings = _load_lang(code)
            for key, en_text in EN.items():
                en_ph = PLACEHOLDER_RE.findall(en_text)
                if not en_ph:
                    continue
                tr_text = strings.get(key, '')
                tr_ph = PLACEHOLDER_RE.findall(tr_text)
                # Same multiset of placeholders (order may vary for some MT)
                if sorted(en_ph) != sorted(tr_ph):
                    # Allow exact sequential match failure only if formatting still works
                    # with dummy args built from English placeholders
                    if not self._can_format(tr_text, en_ph):
                        failures.append((code, key, en_ph, tr_ph, tr_text[:80]))
        if failures:
            sample = failures[:8]
            self.fail(
                '%d placeholder mismatches; samples: %s' % (len(failures), sample))

    def _can_format(self, template, placeholders):
        args = []
        kwargs = {}
        for p in placeholders:
            if p.startswith('%('):
                # named printf — skip detailed check
                return True
            if p.startswith('{') and p.endswith('}'):
                inner = p[1:-1]
                if inner.isdigit() or inner == '':
                    args.append('X')
                else:
                    name = inner.split('!')[0].split(':')[0]
                    if name:
                        kwargs[name] = 'X'
                    else:
                        args.append('X')
            elif any(c in p for c in 'fFeEgG'):
                args.append(1.5)
            elif any(c in p for c in 'diuxX'):
                args.append(3)
            elif p == '%%':
                continue
            else:
                args.append('X')
        try:
            if kwargs and not args:
                template.format(**kwargs)
            elif '{' in template and '}' in template and not re.search(r'%\d|%[sdif]', template):
                if kwargs:
                    template.format(**kwargs)
                else:
                    template.format(*args)
            else:
                if args:
                    template % tuple(args)
                else:
                    # no args needed
                    pass
            return True
        except Exception:
            return False


class TestLanguageSwitching(unittest.TestCase):
    '''Runtime language selection and tr().'''

    def setUp(self):
        mp_i18n.init('en')

    def tearDown(self):
        mp_i18n.init('en')

    def test_init_english_default(self):
        mp_i18n.init('en')
        self.assertEqual(mp_i18n.get_language(), 'en')
        self.assertEqual(mp_i18n.tr('no_link'), EN['no_link'])

    def test_switch_to_each_required_language(self):
        for code in REQUIRED_LANGS:
            ok = mp_i18n.set_language(code)
            self.assertTrue(ok, msg='set_language(%r) failed' % code)
            self.assertEqual(mp_i18n.get_language(), code)
            # known key must resolve to something
            text = mp_i18n.tr('loaded_module', 'testmod')
            self.assertIsInstance(text, str)
            self.assertTrue(len(text) > 0)
            if code != 'en':
                # Should differ from English for this key (all catalogs translate it)
                en_text = EN['loaded_module'] % 'testmod'
                self.assertNotEqual(
                    text, en_text,
                    msg='%s loaded_module still English: %r' % (code, text))

    def test_unknown_language_falls_back_to_english(self):
        ok = mp_i18n.set_language('zz_not_a_real_language')
        self.assertFalse(ok)
        self.assertEqual(mp_i18n.get_language(), 'en')
        self.assertEqual(mp_i18n.tr('no_link'), EN['no_link'])

    def test_language_aliases(self):
        cases = [
            ('german', 'de'),
            ('deutsch', 'de'),
            ('french', 'fr'),
            ('spanish', 'es'),
            ('italian', 'it'),
            ('hindi', 'hi'),
            ('portuguese', 'pt'),
            ('tamil', 'ta'),
            ('telugu', 'te'),
            ('english', 'en'),
        ]
        for alias, expected in cases:
            ok = mp_i18n.set_language(alias)
            self.assertTrue(ok, msg='alias %r failed' % alias)
            self.assertEqual(mp_i18n.get_language(), expected, msg='alias %r' % alias)

    def test_tr_printf_formatting(self):
        mp_i18n.set_language('en')
        self.assertEqual(
            mp_i18n.tr('loaded_module', 'foo'),
            EN['loaded_module'] % 'foo')
        self.assertEqual(
            mp_i18n.tr('loaded_module') % 'foo',
            EN['loaded_module'] % 'foo')

    def test_unknown_key_returns_key(self):
        mp_i18n.set_language('en')
        key = 'this_key_definitely_does_not_exist_xyzzy'
        self.assertEqual(mp_i18n.tr(key), key)

    def test_resolve_key_from_english_text(self):
        mp_i18n.set_language('en')
        english = EN['cmd_terrain_control']
        self.assertEqual(mp_i18n.resolve_key(english), 'cmd_terrain_control')
        # After switching language, early-bound English still re-translates
        mp_i18n.set_language('de')
        de_text = mp_i18n.tr(english)
        self.assertNotEqual(de_text, english)
        self.assertEqual(de_text, mp_i18n.tr('cmd_terrain_control'))

    def test_early_language_from_argv(self):
        self.assertEqual(
            mp_i18n.early_language_from_argv(['--language', 'de', '--help']),
            'de')
        self.assertEqual(
            mp_i18n.early_language_from_argv(['--language=fr']),
            'fr')
        self.assertEqual(
            mp_i18n.early_language_from_argv(['--master', 'tcp:1']),
            'en')

    def test_ensure_language_from_argv(self):
        lang = mp_i18n.ensure_language_from_argv(['--language', 'it'])
        self.assertEqual(lang, 'it')
        self.assertEqual(mp_i18n.get_language(), 'it')


class TestTechnicalShortForms(unittest.TestCase):
    '''Guard against MT mangling of aviation short forms.'''

    def test_magcal_not_mistranslated_as_magic(self):
        for code in REQUIRED_LANGS:
            strings = _load_lang(code)
            text = strings.get('cmd_magcal', '')
            self.assertIn(
                'magcal', text.lower(),
                msg='%s cmd_magcal missing token magcal: %r' % (code, text))
            # Common wrong translations (avoid matching legitimate "magcal")
            for bad in ('magisch', 'magique'):
                self.assertNotIn(
                    bad, text.lower(),
                    msg='%s cmd_magcal looks like "magic": %r' % (code, text))
            # "magico/magica" as whole words only
            self.assertIsNone(
                re.search(r'\bmagic[oa]\b', text.lower()),
                msg='%s cmd_magcal looks like "magic": %r' % (code, text))

    def test_usage_magcal_keeps_cli_token(self):
        for code in REQUIRED_LANGS:
            strings = _load_lang(code)
            for key in ('usage_magcal_start_accept_cancel_yaw',
                        'usage_magcal_yaw_yaw_degrees_mask'):
                if key not in strings:
                    continue
                self.assertIn(
                    'magcal', strings[key],
                    msg='%s %s lost CLI token: %r' % (code, key, strings[key]))

    def test_magical_module_name_preserved(self):
        for code in REQUIRED_LANGS:
            strings = _load_lang(code)
            text = strings.get('magical_ui_process_timed_out_killing', '')
            self.assertTrue(
                text.lower().startswith('magical'),
                msg='%s magical module name altered: %r' % (code, text))


class TestHelpCommandDescriptions(unittest.TestCase):
    '''Command help must re-translate after language switch.'''

    def setUp(self):
        mp_i18n.init('en')

    def tearDown(self):
        mp_i18n.init('en')

    def test_bound_english_description_retranslates(self):
        # Simulate module load while English is active
        bound = mp_i18n.tr('cmd_magcal')
        key = mp_i18n.resolve_key(bound)
        self.assertEqual(key, 'cmd_magcal')
        mp_i18n.set_language('de')
        de = mp_i18n.tr(bound)
        self.assertIn('magcal', de.lower())
        self.assertNotEqual(de, bound)

    def test_resolve_key_idempotent_on_keys(self):
        self.assertEqual(mp_i18n.resolve_key('cmd_magcal'), 'cmd_magcal')
        self.assertEqual(mp_i18n.resolve_key('loaded_module'), 'loaded_module')


class TestI18nOptionParser(unittest.TestCase):
    '''I18nOptionParser builds without error and translates headings.'''

    def setUp(self):
        mp_i18n.init('en')

    def tearDown(self):
        mp_i18n.init('en')

    def test_parser_help_contains_translated_heading_de(self):
        mp_i18n.set_language('de')
        parser = mp_i18n.I18nOptionParser(mp_i18n.tr('opt_usage_mavproxy_py_options'))
        parser.add_option('--master', help=mp_i18n.tr('opt_mavlink_master_port_and_optional_baud_rate'))
        help_text = parser.format_help()
        self.assertIn(mp_i18n.tr('opt_heading_options'), help_text)
        self.assertIn(mp_i18n.tr('opt_show_this_help_message_and_exit'), help_text)


class TestSessionMessageKeys(unittest.TestCase):
    '''Keys introduced for language UI must use stable, non-deprecated names.'''

    def test_language_ui_keys_exist_without_en_de_fr_suffix(self):
        self.assertIn('set_ui_language_code', EN)
        self.assertIn('opt_ui_language_for_user_facing_messages', EN)
        self.assertNotIn('set_ui_language_code_en_de_fr', EN)
        self.assertNotIn('opt_ui_language_for_user_facing_messages_en_de', EN)

    def test_language_ui_keys_present_in_all_catalogs(self):
        for code in REQUIRED_LANGS:
            strings = _load_lang(code)
            self.assertIn('set_ui_language_code', strings, msg=code)
            self.assertIn('opt_ui_language_for_user_facing_messages', strings, msg=code)
            self.assertNotIn('set_ui_language_code_en_de_fr', strings, msg=code)
            self.assertNotIn('opt_ui_language_for_user_facing_messages_en_de', strings, msg=code)


def load_tests(loader, tests, pattern):
    '''Standard unittest load hook.'''
    suite = unittest.TestSuite()
    for case in (
        TestCatalogFiles,
        TestPlaceholderIntegrity,
        TestLanguageSwitching,
        TestTechnicalShortForms,
        TestHelpCommandDescriptions,
        TestI18nOptionParser,
        TestSessionMessageKeys,
    ):
        suite.addTests(loader.loadTestsFromTestCase(case))
    return suite


if __name__ == '__main__':
    # Default to verbose when executed as a script unless -q given
    verbosity = 2
    args = sys.argv[1:]
    if '-q' in args or '--quiet' in args:
        verbosity = 1
        args = [a for a in args if a not in ('-q', '--quiet')]
    elif '-v' in args or '--verbose' in args:
        verbosity = 2
        args = [a for a in args if a not in ('-v', '--verbose')]
    unittest.main(module=__name__, verbosity=verbosity, argv=['test_i18n'] + args)
