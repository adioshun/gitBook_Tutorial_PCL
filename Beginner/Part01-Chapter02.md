Down sampling
====================

점군 Resampling은 목적에 따라서 점군의 수를 줄이거나 늘리는것을 의미 합니다.

- 점군을 줄이는 것을 다운샘플링이라 하며, 연산 부하등의 목적으로 수행 합니다.
  - Voxel Grid

- 점군을 늘리는 것을 업샘플링이라 하며, 탐지/식별 정확도 향상 등을 목적으로 수행 합니다.
  - surface reconstruction

본 챕터에서는 다운샘플링 중 voxelization을 통해서 voxel Grid를 생성 하는 부분만을 다루고 있습니다.

> 참고 복셀은 이미지를 구성하는 1x1 최소 단위를 pixel(picture element)이라고 하듯이, 3D 공간에서의 1x1x1 최소 단위를 Voxel(Volume + Pixel)이라고 합니다. 단위는 고정되어 있지 않고 사용자가 정의 가능 합니다. 


