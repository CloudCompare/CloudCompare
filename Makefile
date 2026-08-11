.PHONY: validate envcheck

envcheck:
	python -m geodesic.tools.envcheck

validate:
	python -m geodesic.tools.validate
