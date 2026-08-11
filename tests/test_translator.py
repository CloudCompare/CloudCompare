import pytest

from geodesic.translator import Translator


def test_translator_builds_anchor_from_record():
    anchor = Translator().anchor_from_record({"id": 7, "coordinates": (1, 2), "label": "x"})
    assert anchor.name == "7"
    assert anchor.coordinates == (1.0, 2.0)
    assert anchor.metadata["label"] == "x"


def test_translator_builds_manifold_from_records():
    manifold = Translator().manifold_from_records([
        {"name": "a", "coordinates": (0, 0)},
        {"name": "b", "coordinates": (3, 4)},
    ])
    assert manifold.distance("a", "b") == 5


def test_translator_requires_coordinates():
    with pytest.raises(ValueError):
        Translator().anchor_from_record({"name": "missing"})


def test_encode_text_is_deterministic():
    assert Translator().encode_text("ab 12") == (0.4, 0.4, 0.2)


def test_geodesic_translator_pressure_response():
    torch = pytest.importorskip("torch")
    from geodesic.translator import GeodesicTranslator, TranslatorState

    translator = GeodesicTranslator(gamma_max=0.1)
    q = torch.zeros(1, 2, 3)
    k = torch.zeros(1, 2, 3)
    states = []
    corrections = []
    previous = None
    for pressure_scale in [0.0, 0.2, 0.6, 1.0, 1.5]:
        distances = torch.full((1, 2, 2), pressure_scale)
        output = translator(q + pressure_scale, k, distances, previous_queries=previous)
        states.append(output.translator_state)
        corrections.append(float(output.correction.mean()))
        previous = q
    assert states[0] == TranslatorState.NORMAL
    assert states[-1] in {TranslatorState.PRESSURE, TranslatorState.REJECT}
    assert corrections == sorted(corrections)
