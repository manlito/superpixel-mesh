#include <superpixel_mesh/meshing_iteration_callback.hpp>

namespace superpixel_mesh {

void MeshingIterationCallback::SetCallback(unsigned long callback_pointer) {
  has_callback = true;
  this->callback_pointer = callback_pointer;
}

bool MeshingIterationCallback::HasCallback() const { return has_callback; }

void MeshingIterationCallback::operator()(
    const MeshingIterationProgress &progress) {
  iteration_progress = progress;
  ((void (*)())callback_pointer)();
}

const MeshingIterationProgress &
MeshingIterationCallback::GetIterationProgress() const {
  return iteration_progress;
}

} // namespace superpixel_mesh
