"""Runtime configuration flags are tested through option().

g_conf stores option(Flag): the flag without its Runtime_Configuration tag
(bit 29). `conf & Flag` also tests bit 29; it only gives the right answer
while that bit is never stored, so every test goes through option(), and the
only path that stores operator options (update_global_config) strips the tag.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def config_flags():
    text = (ROOT / 'Core/Inc/fcstructs.h').read_text()
    body = text[text.index('typedef enum flight_message'):text.index('} fc_msg;')]
    flags = {m.group(1): int(m.group(2) or 0)
             for m in re.finditer(r'\n\s*(\w+)\s*=\s*Runtime_Configuration\s*\|\s*(?:1u|\(1u << (\d+)\))', body)}
    # Message codes and bounds, not flags stored in g_conf.
    for name in ('Revoke_Option', 'User_Option_Bound', 'Abortion_Thresholds', 'Reinit_Thresholds'):
        flags.pop(name, None)
    return flags


def sources():
    for folder in ('Core/Src', 'Core/Inc', 'Core/Test', 'FileX/App'):
        for path in sorted((ROOT / folder).glob('*.[ch]')):
            if path.name != 'fcstructs.h':
                yield path, path.read_text(errors='replace')


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', lambda m: re.sub(r'[^\n]', ' ', m.group(0)), text, flags=re.S)
    return re.sub(r'//[^\n]*', '', text)


class ConfigFlagMasks(unittest.TestCase):
    def test_flags_are_tagged_and_the_tag_is_not_an_option_bit(self):
        flags = config_flags()
        self.assertIn('Retired_Eval_Focused', flags)
        self.assertIn('Vigilant_Mode', flags)
        self.assertIn('Using_Ascent_KF', flags)
        self.assertTrue(all(bit < 28 for bit in flags.values()), flags)

    def test_every_bitwise_test_of_a_flag_uses_option(self):
        names = '|'.join(sorted(config_flags()))
        raw_test = re.compile(r'&\s*\(?\s*(%s)\b' % names)
        offenders = []
        for path, text in sources():
            for lineno, line in enumerate(strip_comments(text).splitlines(), 1):
                for m in raw_test.finditer(line):
                    offenders.append(f'{path.relative_to(ROOT)}:{lineno}: {line.strip()}')
        self.assertEqual(offenders, [], 'test g_conf flags as conf & option(Flag)')

    def test_no_raw_focused_test_left(self):
        dist = (ROOT / 'Core/Src/distribution.c').read_text()
        self.assertNotIn('Eval_Focused', dist)

    def test_operator_options_never_store_the_tag(self):
        recovery = (ROOT / 'Core/Src/recovery.c').read_text()
        body = recovery[recovery.index('static inline void update_global_config('):]
        body = body[:body.index('\n}\n')]
        self.assertIn('fu32 raw = incoming & ~Revoke_Option;', body)
        self.assertIn('g_conf |= raw;', body)
        types = (ROOT / 'Core/Inc/fcstructs.h').read_text()
        self.assertRegex(types, r'Revoke_Option = Runtime_Configuration \| \(1u << 28\)')
        # Every other store of a flag into g_conf goes through option().
        store = re.compile(r'g_conf\s*(?:\|=|&=\s*~)\s*(?!option\(|raw\b)[^;]*;')
        offenders = [f'{p.relative_to(ROOT)}: {m.group(0)}' for p, t in sources()
                     for m in store.finditer(strip_comments(t)) if 'option(' not in m.group(0) and m.group(0) != 'g_conf |= raw;']
        self.assertEqual(offenders, [])


if __name__ == '__main__':
    unittest.main()
