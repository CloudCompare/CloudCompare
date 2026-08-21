PYTHON ?= $(shell test -x .venv/bin/python && echo .venv/bin/python || echo python3)

.PHONY: validate envcheck test board install-geodesic

envcheck:
	$(PYTHON) -m geodesic.tools.envcheck

validate:
	$(PYTHON) -m geodesic.tools.validate

test:
	$(PYTHON) -m pytest tests

board:
	$(PYTHON) -m geodesic.tools.board --host 0.0.0.0 --port 8765

install-geodesic:
	$(PYTHON) -m pip install -r requirements-dev.txt
