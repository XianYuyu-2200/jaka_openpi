"""Data transforms for the JAKA Mini 2 + native six-axis O6 interface."""

import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model


def _parse_image(image: np.ndarray) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).clip(0, 255).astype(np.uint8)
    if image.ndim == 3 and image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    if image.ndim != 3 or image.shape[-1] != 3:
        raise ValueError(f"expected an RGB image, got shape {image.shape}")
    return image


@dataclasses.dataclass(frozen=True)
class JakaMini2Inputs(transforms.DataTransformFn):
    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
        state = np.asarray(data["state"], dtype=np.float32)
        actions = data.get("actions")
        if state.shape[-1] != 12:
            raise ValueError(f"JAKA Mini 2 state must have 12 dimensions, got {state.shape}")
        base_image = _parse_image(data["image"]["base_0_rgb"])
        wrist_image = _parse_image(data["image"]["left_wrist_0_rgb"])
        return_data = {
            "state": state,
            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": wrist_image,
                "right_wrist_0_rgb": np.zeros_like(base_image),
            },
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.True_ if self.model_type == _model.ModelType.PI0_FAST else np.False_,
            },
        }
        if actions is not None:
            actions = np.asarray(actions, dtype=np.float32)
            if actions.shape[-1] != 12:
                raise ValueError(f"JAKA Mini 2 actions must have 12 dimensions, got {actions.shape}")
            return_data["actions"] = actions
        if "prompt" in data:
            return_data["prompt"] = data["prompt"]
        return return_data


@dataclasses.dataclass(frozen=True)
class JakaMini2Outputs(transforms.DataTransformFn):
    def __call__(self, data: dict) -> dict:
        actions = np.asarray(data["actions"])
        if actions.shape[-1] < 12:
            raise ValueError(f"policy output has fewer than 12 action dimensions: {actions.shape}")
        return {"actions": actions[..., :12]}
