# An Introduction to Asyncio

`asyncio` is a standard Python library used to write concurrent code using asynchronous
syntax. This library is similar to a predecessor library known as Twisted, which
was used heavily in our repository for nearly a decade before it was removed from
most files in our repository.

:::{note}
If you are planning to work with asynchronous code and have not already set up
an IDE or editor with a language server, you are **highly recommended** to do so.
It is very common to make mistakes in asynchronous programming (adding `await` somewhere
it shouldn't be, using the wrong `asyncio` method, etc.) and having to run your
program each time to figure out if you made any of these mistakes is not ideal.

A type checker in your IDE can catch these mistakes immediately. Setting up a language
server can take as little as 10 minutes, and is very worth it in the long run.
:::

## Migrating from Twisted to Asyncio

In Twisted, coroutines were specified through the following syntax:

```python
from twisted.internet import defer
from axros.util import cancellableInlineCallbacks

class Example():
    @defer.inlineCallbacks()
    def example(self):
        yield some_sleep_helper(2)
        yield other_coroutine()
        defer.returnValue("Done!")

    # or
    @cancellableInlineCallbacks
    def other_example(self):
        yield other_coroutine()
```

The second decorator, created by a former MIL member, was used to enhance `defer.inlineCallbacks`
by allowing for the cancellation of nested coroutines. It's primary function was
very similar to that of the standard `defer.inlineCallbacks` method.

Now, we can use `asyncio` to achieve similar results:
```python
import asyncio

class Example():
    async def example(self):
        await asyncio.sleep(2)
        await other_coroutine()
        return "Done!"

    async def other_example(self):
        await other_coroutine()
```

You'll notice some nice syntax improvements: the loss of a standard decorator to
mark a method as being a coroutine, the use of special keywords made specifically
for `async`, the ability to call `return` in a coroutine, and some helper functions
provided by the standard `asyncio` library.

## Coroutines, Tasks, and Futures
There are three notable parts of `asyncio` you should be familiar with:
* **Coroutines**: These are asynchronous functions. You can write these using
  the `async def` syntax. Coroutines can be called with the `await` keyword to
  start the execution of the coroutine.
* **Futures**: A future is an object that is _immediately returned_ with the expectation
  that it will have a populated value at a later date. You can `await` futures
  to pause execution of the current method until the future has a result. In Python,
  futures are reresented through the {class}`asyncio.Future` object.
* **Tasks**: Tasks are high-level futures that are easier to work with in client
  code. When writing high-level code (missions, for example), you should try to use
  tasks, not futures. Tasks are represented in Python through the {class}`asyncio.Task`
  class and are typically spawned through {func}`asyncio.create_task`.
* **Awaitables**: This name refers to all three of the above, because all three of
  the above can have the `await` keyword called on them.

For some great examples of these three different classes, check out the Python
documentation on {ref}`Coroutines <py:coroutine>`.

## Help from `asyncio`
The standard library `asyncio` will provide you a lot of help when working with
awaitables. Let's take a look at some methods that you may want to use:

* {func}`asyncio.sleep`: This coroutine pauses the parent coroutine for a specific
  number of seconds. Useful!
* {func}`asyncio.wait_for`: Sets a time limit for a coroutine: if the coroutine
  does not finish within the time limit, then an exception is raised.
* {func}`asyncio.wait`: Waits for a bunch of coroutines to finish in a certain manner.
  This function can either wait for the first coroutine to finish, or it can
  wait for all coroutines to finish.
* {func}`asyncio.shield`: Prevents a coroutine from being cancelled.
* {func}`asyncio.gather`: Run coroutines concurrently (ie, together). This can speed
  up execution time by a great amount, but be weary of issues that may be caused
  by running two coroutines at the same time (what if the other coroutine needs a resource
  that can only be providd by the other coroutine?).
* {func}`asyncio.create_task`: Creates a {class}`asyncio.Task` that begins the execution
  of a coroutine all by itself. Similar to running a function in a thread. Note
  that this method is not a coroutine, and therefore should not be called with
  `await`.
* {func}`asyncio.run`: Synchronous method that provides an entrypoint into an asynchronous
  program. This should always be called if you need to call an asynchronous method
  directly when starting a program. This should rarely be used if the program has
  _already_ started.

## Yielding Control
When writing asynchronous functions, you need to be careful to avoid hogging the
event loop. If you do this, you are going to prevent other coroutines from running
at the same time.

For example, this coroutine:
```python
async def uh_oh():
    while True:
        pass
```
would block all other coroutines on the event loop. This coroutine need to yield
control.

The simplest way to do this is to use {func}`asyncio.sleep`, even if the argument
is zero. This coroutine will not end up blocking the event loop:
```python
import asyncio

async def uh_oh():
    while True:
        asyncio.sleep(0) # Yield control to other coroutines!
```

## Typing `asyncio` Code
As our repository becomes more and more typed, you may encounter a challenge where
you need to type an asynchronous part of a program. How do you go about this?

### Asynchronous Functions
For most asynchronous functions, you can mark the return type as whatever
the coroutine itself returns after being scheduled.

```python
import asyncio

async def test(a: int, b: int) -> int:
    await asyncio.sleep(2)
    return a + b
```

However, if you want to type the actual asynchronous function itself, you will
need to type the coroutine itself. You can either use {class}`collections.abc.Coroutine`
or {class}`typing.Coroutine` for this. This type takes three generics. Usually,
the first two can be {class}`typing.Any` - the third generic is the return type
of the coroutine.

```python
import asyncio
from collections.abc import Coroutine
from typing import Any

async def test(a: int, b: int) -> int:
    await asyncio.sleep(2)
    return a + b

def gimme_a_test() -> Coroutine[Any, Any, int]:
    return test(1, 2)
```

### Futures and Tasks
Both {class}`asyncio.Future` and {class}`asyncio.Task` are generics, each taking
one type: the result type of the future or task. While you are not required to type
these classes as generics, its recommended for easier understanding later.

```python
import asyncio

async def test() -> asyncio.Future[int]:
    fut = asyncio.Future()
    fut.set_result(3)
    return fut
```

### {mod}`axros`
{mod}`axros` now has a lot of great typing support for its asynchronous pieces.
{class}`axros.Subscriber` and {class}`axros.Publisher` are generics which accept
the message type that they are receiving and publishing.

```python
import asyncio
from axros import NodeHandle
from geometry_msgs.msg import Point, PointStamped

async def main():
    nh = NodeHandle.from_argv("my_special_node")
    await nh.setup()
    pub = nh.advertise("special_point", Point)
    pub.publish(PointStamped()) # This is a type error
    await nh.shutdown()

asyncio.run(main())
```

## Using `uvloop`
One extension that can be used on top of `asyncio` is an extension known as `uvloop`.
When installed, this extension greatly speeds up the event loop itself, which helps
the loop to move through coroutines faster. This can make your programs much more
performant.

Therefore, whenever you use {func}`asyncio.run`, you should also be using `uvloop.install`.
```python
import asyncio
import uvloop

async def main():
    ...

if __name__ == "__main__":
    asyncio.run(main())
    uvloop.install()
```
