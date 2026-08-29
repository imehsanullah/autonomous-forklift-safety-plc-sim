.PHONY: install demo headless gifs

install:
	uv sync --extra dev --extra sim

demo:
	uv run forklift-sim --plc emulated --scenario demonstration

headless:
	uv run forklift-sim --plc emulated --scenario protective_stop --headless

gifs:
	uv run forklift-sim --plc emulated --scenario person_crossing --headless --record-gif assets/person-crossing.gif
	uv run forklift-sim --plc emulated --scenario fault_showcase --headless --record-gif assets/fault-response.gif
