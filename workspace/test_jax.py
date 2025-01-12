import jax
import jax.numpy as jnp

# Check available devices
print("Available devices:", jax.devices())

# Check the default device
array = jnp.array([1.0, 2.0, 3.0])
print("Array device:", array.device)
