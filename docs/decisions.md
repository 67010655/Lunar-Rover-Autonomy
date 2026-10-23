# บันทึกการตัดสินใจและสถานะ

## 2026-10-24 — เริ่ม repo และ simulation baseline

**การอนุมัติ:** ผู้ใช้สร้าง repo `67010655/Lunar-Rover-Autonomy` และแจ้ง “สามารถเริ่มงานได้เลย ผมอนุมัติทุกอย่าง” หลังแผน README, experiment plan และ simulation รอบแรก จึงเริ่มเชื่อม repo พัฒนา ทดสอบ และบันทึกงานรอบนี้

**ขอบเขตปัจจุบัน:** simulation-first; P0 บนสนามสังเคราะห์ พร้อม closed-loop execution และตำแหน่งจริง/ประมาณแยกกัน เพิ่มโหมด odometry/fixes อย่างง่ายเพื่อทดสอบ interface เท่านั้น

**ที่มาของขอบเขต:** ใช้คำชี้แจงหลังประชุมที่ผู้ใช้แจ้งในบทสนทนา ประกอบกับแนวคิด P0/P1/P2 จากเอกสารเดิม ไม่ได้ถือว่างบ กำหนดเวลา หรือสมมติฐานในเอกสารก่อนประชุมเป็นข้อกำหนดล่าสุดโดยอัตโนมัติ

### เลือกใช้ในต้นแบบนี้

- Python 3.12+, core planner/controller ใช้ standard library; Matplotlib/Pillow สำหรับภาพและ GIF
- แผนที่ Cartesian 2D หน่วย m, seed ชัดเจน, JSON input, CSV/JSON output
- Circular body + (v, yaw-rate) abstraction; ไม่ตรึง steering/actuator architecture ของรถจริง
- A* 8 ทิศ พร้อม hard collision/clearance constraint; P1/P2 ยังไม่ implement
- ไม่คัดลอกเอกสารรายงาน/สไลด์เดิมทั้งหมดเข้า public repo และไม่เปลี่ยนไฟล์ต้นฉบับ
- รักษา MIT license ที่เจ้าของ repo สร้างไว้ ไม่เพิ่ม license ให้ CAD/ข้อมูลภายนอกที่ยังไม่ได้นำมาใช้

### ยังไม่ยืนยัน

- ตัว estimator/เซนเซอร์สำหรับการเปรียบเทียบขั้นสุดท้าย รวม Visual หรือ LiDAR SLAM
- GNSS simulation noise/bias, latency, dropouts, sensor rates และ sensor fusion
- Lunar ROI/dataset, terrain/communication costs, normalization, thresholds และ evaluation splits
- จำนวน repeated trials และกำหนดส่งจริง
- ขนาดรถ CAD/motor sizing, steering, power และ BOM ภายใต้งบซื้อ 6,000 บาท

### สิ่งที่ไม่ควรอ้างจากโค้ดรอบนี้

- ไม่มี SLAM, encoder/IMU fusion, lunar DEM pipeline หรือ radio propagation model ที่ทำงานแล้ว
- synthetic fixes ไม่ใช่การรับ GNSS บนดวงจันทร์
- collision monitor เป็น evaluator ใน simulator ไม่ใช่ obstacle sensor หรือระบบหยุดรถจริง
- ไม่มี NASA/JPL endorsement และไม่มีการทดสอบ rover hardware
