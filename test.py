import asyncio

import uvloop


async def main():
    count = 0
    while True:
        await asyncio.sleep(1)
        count += 1
        print("*" * count)


if __name__ == "__main__":
    # uvloop.install()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
