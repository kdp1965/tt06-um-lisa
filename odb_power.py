#
# OpenDB script for custom Power for tt_top / user_project_wrapper
#
# Copyright (c) 2023 Sylvain Munaut <tnt@246tNt.com>
# SPDX-License-Identifier: Apache-2.0
#

import os
import sys

import odb
import click

from openlane.common.misc import get_openlane_root
sys.path.insert(0, os.path.join(get_openlane_root(), "scripts", "odbpy"))
from reader import click_odb

@click.option(
    "--macro-x-pos", default=0, type=int, help="X position of the RAM32 macro"
)
@click.command()
@click_odb
def power(reader, macro_x_pos: int):
  # Create ground / power nets
  tech = reader.db.getTech()
  vpwr_net = reader.block.findNet('VPWR')
  vgnd_net = reader.block.findNet('VGND')
  met4 = tech.findLayer('met4')
  vpwr_wire = vpwr_net.getSWires()[0]
  vgnd_wire = vgnd_net.getSWires()[0]
  vpwr_bterm = vpwr_net.getBTerms()[0]
  vgnd_bterm = vgnd_net.getBTerms()[0]
  vpwr_bpin = vpwr_bterm.getBPins()[0]
  vgnd_bpin = vgnd_bterm.getBPins()[0]
  for i in range(3):
    x = macro_x_pos * 1000 + 18280 + i * 153600
    odb.dbSBox_create(vpwr_wire, met4, x, 11880, x + 1600, 144120, "STRIPE")
    odb.dbBox_create(vpwr_bpin, met4, x, 11880, x + 1600, 144120)
  for i in range(2):
    x = macro_x_pos * 1000 + 95080 + i * 153600
    odb.dbSBox_create(vgnd_wire, met4, x, 11880, x + 1600, 144120, "STRIPE")
    odb.dbBox_create(vgnd_bpin, met4, x, 11880, x + 1600, 144120)


if __name__ == "__main__":
  power()
