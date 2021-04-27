#!/usr/bin/env python
import asyncio
import websockets
import sys

import math
import time
import json
from json import JSONEncoder
import numpy
from examples.drone import init, step, update

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, numpy.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)

async def consumer_handler(websocket, path):
    while True:
        out = await websocket.recv()
        data = json.loads(out)
        ri = data['ri']['x'], data['ri']['z'], data['ri']['y'],
        rf = data['rf']['x'], data['rf']['z'], data['rf']['y'],
        wp = data['wp']['x'], data['wp']['z'], data['wp']['y'],
        update(dict(ri=ri, wp=wp, rf=rf))

async def producer_handler(websocket, path):
    state, params = init()
    count = 0
    print('Initializing')
    while True:
        print(count)
        count += 1 
        package = dict(S=state)
        raw =  json.dumps(package, cls=NumpyArrayEncoder)
        await websocket.send(raw)
        await asyncio.sleep(0.01)
        state = step(state)

async def handler(websocket, path):
    consumer_task = asyncio.ensure_future(
        consumer_handler(websocket, path))
    producer_task = asyncio.ensure_future(
        producer_handler(websocket, path))
    done, pending = await asyncio.wait(
        [consumer_task, producer_task],
        return_when=asyncio.FIRST_COMPLETED,
    )
    for task in pending:
        task.cancel()


if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--freq', default=60, type=float)
    args = parser.parse_args()

    start_server = websockets.serve(handler, "localhost", 8765)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
