import asyncio
from makcu import create_async_controller, MouseButton

async def main():
    # Auto-connect with context manager
    async with await create_async_controller(debug=True) as makcu:
        # Parallel operations
        await asyncio.gather(
            makcu.move(100, 0),
            makcu.click(MouseButton.LEFT),
            makcu.scroll(-1)
        )
        
        # Human-like clicking
        await makcu.click_human_like(MouseButton.RIGHT, count=3)

asyncio.run(main())