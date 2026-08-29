import numpy as np
import pytest

from openpi.models import model as _model
from openpi.policies import jaka_mini2_policy


def test_jaka_inputs_map_two_images_and_12d_state() -> None:
    transform = jaka_mini2_policy.JakaMini2Inputs(_model.ModelType.PI05)
    result = transform(
        {
            "state": np.zeros(12, dtype=np.float32),
            "image": {
                "base_0_rgb": np.zeros((8, 8, 3), dtype=np.uint8),
                "left_wrist_0_rgb": np.zeros((3, 8, 8), dtype=np.uint8),
            },
            "actions": np.zeros((16, 12), dtype=np.float32),
            "prompt": "test",
        }
    )
    assert result["state"].shape == (12,)
    assert result["actions"].shape == (16, 12)
    assert result["image"]["left_wrist_0_rgb"].shape == (8, 8, 3)
    assert result["image_mask"]["right_wrist_0_rgb"] == np.False_


def test_jaka_outputs_slice_pretrained_width() -> None:
    result = jaka_mini2_policy.JakaMini2Outputs()({"actions": np.zeros((16, 32), dtype=np.float32)})
    assert result["actions"].shape == (16, 12)


def test_jaka_inputs_reject_wrong_dimension() -> None:
    transform = jaka_mini2_policy.JakaMini2Inputs(_model.ModelType.PI05)
    with pytest.raises(ValueError, match="12 dimensions"):
        transform(
            {
                "state": np.zeros(8),
                "image": {
                    "base_0_rgb": np.zeros((8, 8, 3), dtype=np.uint8),
                    "left_wrist_0_rgb": np.zeros((8, 8, 3), dtype=np.uint8),
                },
            }
        )
