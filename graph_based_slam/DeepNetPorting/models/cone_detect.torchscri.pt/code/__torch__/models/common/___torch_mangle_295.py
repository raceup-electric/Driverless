class ImplicitM(Module):
  __parameters__ = ["implicit", ]
  __buffers__ = []
  implicit : Tensor
  training : bool
  _is_full_backward_hook : Optional[bool]
