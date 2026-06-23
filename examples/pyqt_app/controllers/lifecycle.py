from __future__ import annotations


def shutdown_pages(stack) -> None:
    for index in range(stack.count()):
        page = stack.widget(index)
        shutdown = getattr(page, "shutdown", None)
        if callable(shutdown):
            shutdown()
