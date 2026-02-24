# -*- coding: utf-8 -*-
# Copyright (c) 2014-2022 Richard Hull and contributors
# See LICENSE.rst for details.
import logging
import sys
# Path to get fonts
from pathlib import Path

# ImageFont for drawing on device
from PIL import ImageFont

import luma.core
from luma.core import cmdline, error
from luma.core.render import canvas


# logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)-15s - %(message)s'
)
# ignore PIL debug messages
logging.getLogger('PIL').setLevel(logging.ERROR)

BASE_PATH = Path(__file__).resolve().parent


def display_settings(device, args):
    """
    Display a short summary of the settings.

    :rtype: str
    """
    iface = ''
    display_types = cmdline.get_display_types()
    if args.display not in display_types['emulator']:
        iface = f'Interface: {args.interface}\n'

    lib_name = cmdline.get_library_for_display_type(args.display)
    if lib_name is not None:
        lib_version = cmdline.get_library_version(lib_name)
    else:
        lib_name = lib_version = 'unknown'

    version = f'luma.{lib_name} {lib_version} '\
        f'(luma.core {luma.core.__version__})'

    return f'Version: {version}\nDisplay: {args.display}\n{iface}'\
        f'Dimensions: {device.width} x {device.height}\n{"-" * 60}'


def get_device(actual_args=None):
    """
    Create device from command-line arguments and return it.
    """
    if actual_args is None:
        actual_args = sys.argv[1:]
    parser = cmdline.create_parser(description='luma.examples arguments')
    args = parser.parse_args(actual_args)

    if args.config:
        # load config from file
        config = cmdline.load_config(args.config)
        args = parser.parse_args(config + actual_args)

    # create device
    try:
        device = cmdline.create_device(args)
        print(display_settings(device, args))
        return device

    except error.Error as e:
        parser.error(e)
        return None


class Display:
    def __init__(self):
        self.font_size = 12
        self.font_size_full = 10
        self.margin_y_line = [0, 13, 25, 38, 51]
        self.margin_x_figure = 78
        self.margin_x_bar = 31
        self.bar_width = 52
        self.bar_width_full = 95
        self.bar_height = 8
        self.bar_margin_top = 3
        self.display_height = 64

        self.lines = int(self.display_height / self.font_size)

        self.device = get_device()
        font_path = BASE_PATH.joinpath('fonts', 'DejaVuSansMono.ttf')
        if not font_path.exists():
            print(f'WARNING: font {font_path} not fount!')
        self.font_default = ImageFont.truetype(str(font_path), self.font_size)
        self.font_full = ImageFont.truetype(
            str(font_path), self.font_size_full)

    def draw_text(self, draw, margin_x, line_num, text):
        if not text:
            return
        draw.text((
            margin_x,
            self.margin_y_line[line_num]),
            text,
            font=self.font_default, fill='white')

    def draw_bar(self, draw, line_num, percent):
        if percent >= 100:
            return self.draw_bar_full(draw, line_num)
        top_left_y = self.margin_y_line[line_num] + self.bar_margin_top
        draw.rectangle((
            self.margin_x_bar,
            top_left_y,
            self.margin_x_bar + self.bar_width,
            top_left_y + self.bar_height),
            outline='white')
        draw.rectangle((
            self.margin_x_bar,
            top_left_y,
            self.margin_x_bar + self.bar_width * percent / 100,
            top_left_y + self.bar_height),
            fill='white')

    def draw_bar_full(self, draw, line_num):
        top_left_y = self.margin_y_line[line_num] + self.bar_margin_top
        draw.rectangle((
            self.margin_x_bar,
            top_left_y,
            self.margin_x_bar + self.bar_width_full,
            top_left_y + self.bar_height),
            fill='white')
        draw.text(
            (65, top_left_y - 2), '100 %',
            font=self.font_full, fill='black')

    def display_stats(self, data):
        with canvas(self.device) as draw:
            for num, line in enumerate(data):
                if 'type' not in line:
                    continue
                _type = line['type']
                value = line['value']
                line_num = line['line'] if 'line' in line else num
                if _type in ['percentage', 'int']:
                    self.draw_text(draw, 0, line_num, line['text'])

                    percentage_value = value
                    if 'max' in line:
                        percentage_value = value / line['max'] * 100
                    self.draw_bar(draw, line_num, percentage_value)

                    if _type == 'int':
                        val = f'  {int(value)}' if value >= 10\
                            else f'   {int(value)}'
                    else:
                        val = f' {value:5.1f}%'
                    unit = line['unit']
                    self.draw_text(
                        draw, self.margin_x_figure, line_num, f'{val}{unit}')
                elif _type == 'text':
                    value = line['value']
                    left = line['left'] if 'left' in line else 0
                    self.draw_text(draw, left, line_num, value)
