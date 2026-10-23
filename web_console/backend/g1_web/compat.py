"""Small Python 3.8 compatibility surface; no monkey patches to asyncio."""
import asyncio
import contextvars
import functools


async def to_thread(function, *args, **kwargs):
    loop = asyncio.get_running_loop()
    call = functools.partial(contextvars.copy_context().run, function, *args, **kwargs)
    return await loop.run_in_executor(None, call)
