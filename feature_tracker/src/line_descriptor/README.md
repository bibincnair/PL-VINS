# PL-VINS modified line descriptor

The parent feature_tracker package builds this code against the pinned OpenCV 5
installation. No precompiled library is required. For a standalone library build:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/PL-VINS/.deps/install
cmake --build build
```

The modified LSD settings and line filtering are preserved. Original upstream
source copyright and license notices remain in the implementation files.

Binary descriptors for lines extracted from an image
====================================================