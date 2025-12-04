# Contributing Guidelines

Thanks for checking out the Custom LiDAR Integration project!

### 📌 Contribution Rules
- Keep ROS2 nodes modular and documented.
- Avoid hardcoding serial parameters unless required.
- Test UART data integrity using `hexdump` before submitting changes.
- Maintain consistent logging using `self.get_logger()`.

### 📦 Future Improvements
- Add CRC validation for LiDAR frames.
- Integrate RViz2 visualization support.
- Add unit tests for frame parsing.
