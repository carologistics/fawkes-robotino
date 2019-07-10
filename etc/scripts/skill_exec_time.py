#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.
#

"""
Parse the skill execution times from the log file and store them in a database.
"""

import argparse
from datetime import datetime
import re

skill_start_regex = '.*==>\s+.*\(skill\s+.*\(status S_RUNNING\).*\)'
skill_end_regex = '.*==>\s+.*\(skill\s+.*\(status (S_FINAL|S_FAILED)\).*\)'
skill_name_regex = '.*\(skill\s.*\(name ([\w-]+)\).*'
skill_success_regex = '.*==>\s+.*\(skill\s+.*\(status (\w+)\).*\)'
time_regex = '\w\s([0-9:.]+)\s.*'
time_format = '%H:%M:%S.%f'


class SkillCall:

    def __init__(self, skill_name, skill_args, result, start, end):
        self.skill_name = skill_name
        self.skill_args = skill_args
        self.result = result
        self.start = start
        self.end = end
        assert self.end > self.start

    @classmethod
    def from_lines(cls, start_line, end_line):
        skill_name = SkillCall.parse_name(start_line)
        if skill_name != SkillCall.parse_name(end_line):
            raise Exception(
                'Mismatching lines: "{}", "{}"'.format(start_line, end_line))
        start = SkillCall.parse_time(start_line)
        end = SkillCall.parse_time(end_line)
        result = re.match(skill_success_regex, end_line).group(1)
        return cls(skill_name, [], result, start, end)

    @classmethod
    def parse_name(cls, line):
        return re.match(skill_name_regex, line).group(1)

    @classmethod
    def parse_time(cls, line):
        return datetime.strptime(re.match(time_regex, line).group(1), time_format)

    def get_duration(self):
        return self.end - self.start

    def __str__(self):
        return 'name: "{}", duration: {}, result: {}'.format(self.skill_name, self.get_duration(), self.result)

    def __eq__(self, other):
        return self.skill_name == other.skill_name \
            and  self.skill_args ==  other.skill_args \
            and self.start == other.start \
            and self.end == other.end

    def __ne__(self, other):
        return not (self == other)

    def __lt__(self, other):
        return self.start < other.start and self.end < other.end

    def __le__(self, other):
        return (self < other) or (self == other)

    def __gt__(self, other):
        return self.start > other.start and self.end > other.end

    def __ge__(self, other):
        return (self > other) or (self == other)


class LogfileParser:

    def __init__(self, logfile):
        self.logfile = logfile
        self.skill_calls = []

    def parse_file(self):
        while True:
            start_line = self.find_start()
            if not start_line:
                return
            end_line = self.find_end()
            if not end_line:
                return
            skill_call = SkillCall.from_lines(start_line, end_line)
            print('Parsed skill {}'.format(skill_call))
            self.skill_calls.append(skill_call)

    def find_start(self):
        while True:
            next_line = self.logfile.readline()
            if not next_line:
                return None
            if re.match(skill_start_regex, next_line):
                return next_line

    def find_end(self):
        while True:
            next_line = self.logfile.readline()
            if not next_line:
                return None
            if re.match(skill_end_regex, next_line):
                return next_line


def main():
    parser = argparse.ArgumentParser(description='Skill execution time parser')
    parser.add_argument('logfile', type=argparse.FileType('r'), nargs='+')
    args = parser.parse_args()
    for logfile in args.logfile:
        file_parser = LogfileParser(logfile)
        file_parser.parse_file()


if __name__ == '__main__':
    main()
