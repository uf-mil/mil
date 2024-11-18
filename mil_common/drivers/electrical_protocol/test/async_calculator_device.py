#!/usr/bin/env python3
"""
from __future__ import annotations

import asyncio
from dataclasses import dataclass

# from threading import Event, Thread
from typing import Union

import uvloop
from axros import NodeHandle
from electrical_protocol import AsyncSerialDevice, Packet
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


@dataclass
class RequestAddPacket(Packet, class_id=0x37, subclass_id=0x00, payload_format="<ff"):
    number_one: float
    number_two: float


@dataclass
class RequestSubPacket(Packet, class_id=0x37, subclass_id=0x01, payload_format="<ff"):
    start: float
    minus: float


@dataclass
class AnswerPacket(Packet, class_id=0x37, subclass_id=0x02, payload_format="<f"):
    result: float


class AsyncCalculatorDevice(
    AsyncSerialDevice[Union[RequestAddPacket, RequestSubPacket], AnswerPacket],
):
    def __init__(self, nh: NodeHandle):
        self.nh = nh
        print("1")

        # await asyncio.gather(self.port_topic.setup(), self.start_service.setup(), self.answer_topic.setup())

    async def setup(self):
        # Move the async setup logic to a separate method
        print("Setting up AsyncCalculatorDevice...")

        # Create subscriber, service, and publisher
        self.port_topic = self.nh.subscribe("~port", String, self.port_callback)
        await self.port_topic.setup()
        print(f"Port topic: {self.port_topic.is_running}")
        self.start_service = self.nh.advertise_service("~trigger", Empty, self.trigger)
        await self.start_service.setup()
        print(f"Start service: {self.start_service.is_running}")
        self.answer_topic = self.nh.advertise("~answer", Float32, latching=True)
        print(f"Answer topic: {self.start_service.is_running}")
        await self.answer_topic.setup()
        # Set up the subscriber, publisher, and service asynchronously

        print("AsyncCalculatorDevice setup complete.")

        self.next_packet = asyncio.Event()
        self.i = 0

        self.transport = None
        self.protocol = None

    def port_callback(self, msg: String):
        print("2")
        if self.loop.is_running:
            print("why are you running, but good")
        self.connect(msg.data, 115200, self.loop)

    async def trigger(self, _: EmptyRequest) -> EmptyResponse:
        if self.transport == None:
            print("transport is not with us")
        print("4")
        self.num_one, self.num_two = self.i, 1000 - self.i
        self.i += 1
        self.send_packet(
            RequestAddPacket(number_one=self.num_one, number_two=self.num_two),
        )
        return EmptyResponse()

    def on_packet_received(self, packet) -> None:
        self.answer_topic.publish(Float32(data=packet.result))
        self.next_packet.set()


async def main():
    nh, remaining_args = NodeHandle.from_argv_with_remaining(
        "async_calculator_device", anonymous=True,
    )
    async with nh:
        print("Starting AsyncCalculatorDevice...")
        device = AsyncCalculatorDevice(nh)

        await device.setup()


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
"""
