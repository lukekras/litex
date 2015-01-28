from collections import OrderedDict

from migen.fhdl.std import *
from migen.fhdl.std import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.record import *
from migen.genlib.fsm import FSM, NextState
from migen.genlib.misc import chooser
from migen.flow.actor import EndpointDescription
from migen.flow.actor import Sink, Source
from migen.actorlib.structuring import Converter, Pipeline
from migen.actorlib.fifo import SyncFIFO, AsyncFIFO
from migen.bank.description import *

eth_mtu = 1532
eth_preamble = 0xD555555555555555
buffer_depth = 2**log2_int(eth_mtu, need_pow2=False)

class HField():
	def __init__(self, byte, offset, width):
		self.byte = byte
		self.offset = offset
		self.width = width

mac_header_len = 14
mac_header = {
	"destination_mac_address":	HField(0,  0, 48),
	"source_mac_address":		HField(6,  0, 48),
	"ethernet_type":			HField(12, 0, 16)
}

arp_header_len = 28
arp_header = {
	"hardware_type":			HField( 0,  0, 16),
	"protocol_type":			HField( 2,  0, 16),
	"hardware_address_length":	HField( 4,  0, 8),
	"protocol_address_length":	HField( 5,  0, 8),
	"operation":				HField( 6,  0, 16),
	"source_mac_address":		HField( 8,  0, 48),
	"source_ip_address":		HField(14,  0, 32),
	"destination_mac_address":	HField(18,  0, 48),
	"destination_ip_address":	HField(24,  0, 32)
}

ipv4_header_len = 24
ipv4_header = {
	"version":					HField(0,  0, 4),
	"ihl":						HField(0,  4, 4),
	"dscp":						HField(1,  0, 6),
	"ecn":						HField(1,  6, 2),
	"total_length":				HField(2,  0, 16),
	"identification":			HField(4,  0, 16),
	"flags":					HField(6,  0, 3),
	"fragment_offset":			HField(6,  3, 13),
	"time_to_live":				HField(8,  0, 8),
	"protocol":					HField(9,  0, 8),
	"header_checksum":			HField(10,  0, 16),
	"source_ip_address":		HField(12,  0, 32),
	"destination_ip_address":	HField(16,  0, 32),
	"options":					HField(20,  0, 32)
}
udp_header_len = 8
udp_header = {
	"source_port":				HField( 0,  0, 16),
	"destination_port":			HField( 2,  0, 16),
	"length":					HField( 4,  0, 16),
	"checksum":					HField( 6,  0, 16)
}

# layouts
def _layout_from_header(header):
	_layout = []
	for k, v in sorted(header.items()):
		_layout.append((k, v.width))
	return _layout

def eth_phy_description(dw):
	layout = [
		("data", dw),
		("last_be", dw//8),
		("error", dw//8)
	]
	return EndpointDescription(layout, packetized=True)

def eth_mac_description(dw):
	layout = _layout_from_header(mac_header) + [
		("data", dw),
		("last_be", dw//8),
		("error", dw//8)
	]
	return EndpointDescription(layout, packetized=True)

def eth_arp_description(dw):
	layout = _layout_from_header(arp_header) + [
		("data", dw),
		("last_be", dw//8),
		("error", dw//8)
	]
	return EndpointDescription(layout, packetized=True)

def eth_ipv4_description(dw):
	layout = _layout_from_header(ipv4_header) + [
		("data", dw),
		("last_be", dw//8),
		("error", dw//8)
	]
	return EndpointDescription(layout, packetized=True)

def eth_udp_description(dw):
	layout = _layout_from_header(udp_header) + [
		("data", dw),
		("last_be", dw//8),
		("error", dw//8)
	]
	return EndpointDescription(layout, packetized=True)

