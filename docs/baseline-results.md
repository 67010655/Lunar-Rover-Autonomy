# ผลตรวจ simulation baseline รอบแรก

วันที่: 24 ตุลาคม 2026 — development scenario `synthetic_obstacles_v1`, seed 42, P0 เท่านั้น

## ผลที่สังเกตได้

| โมเดลตำแหน่ง | ผลการวิ่ง | RMSE [m] | เวลาจำลอง [s] | ระยะวิ่งจริง [m] | จำนวน samples |
|---|---|---:|---:|---:|---:|
| ideal | ถึงเป้าหมาย | ประมาณ 0 (5.19e-14 numerical error) | 62.1 | 17.309 | 622 |
| odometry | หยุดก่อนก้าวที่จะชนสิ่งกีดขวาง | 0.130 | 28.8 | 7.911 | 289 |
| fixes | ถึงเป้าหมาย | 0.089 | 62.9 | 17.325 | 630 |

ทั้งสามใช้แผน P0 เดียวกัน ระยะทางวางแผน **17.539 m** ข้อมูลจริงและประมาณใช้ timestamp เดียวกัน โหมด ideal ไม่รับ GT กลับไปแก้ตำแหน่ง แต่คำนวณ odometry ที่ไม่มี noise/bias จากคำสั่งการเคลื่อนที่ จึงคลาดเคลื่อนเพียง floating-point

RMSE ของ odometry คำนวณถึงเวลาหยุดที่ 28.8 s ขณะที่โหมดอื่นวิ่งจบ จึงไม่ควรนำตัวเลข RMSE ในตารางไปสรุปความแม่นยำตลอดภารกิจโดยตรง ไม่มีการคำนวณ success rate หรือความเชื่อมั่นทางสถิติจาก 1 run ต่อโหมด

fixes ใช้ตำแหน่งสังเคราะห์ที่มี Gaussian noise ทุก 1 s และ gain คงที่ ยังไม่มี receiver dynamics, latency, dropout, multipath, IMU fusion หรือ SLAM ผลนี้ตรวจการไหลของข้อมูลและการป้อนกลับ ไม่ได้ยืนยันคุณภาพของ GNSS หรือ SLAM

## เปิดดูและตรวจสอบ

- [ideal: ภาพรวม](../examples/seed42/ideal/overview.png) · [GIF](../examples/seed42/ideal/trajectory.gif) · [metrics](../examples/seed42/ideal/metrics.json)
- [odometry: ภาพรวม](../examples/seed42/odometry/overview.png) · [GIF](../examples/seed42/odometry/trajectory.gif) · [metrics](../examples/seed42/odometry/metrics.json)
- [fixes: ภาพรวม](../examples/seed42/fixes/overview.png) · [GIF](../examples/seed42/fixes/trajectory.gif) · [metrics](../examples/seed42/fixes/metrics.json)

ในภาพ เส้นประเทาคือแผน P0 สีน้ำเงินคือตำแหน่งจริง และสีส้มคือตำแหน่งประมาณ พื้นดำคือ hard obstacles ส่วนสีเทาอ่อนคือบริเวณที่ planner กันไว้เพื่อ clearance เส้นสีแดงในกราฟขวาคือขนาดความคลาดเคลื่อนตำแหน่งตามเวลาจำลอง

ทุก run มี config snapshot, trajectory CSV และ manifest ที่ชี้ source revision `590305f` พร้อม source/output SHA-256 โดย source อยู่ในสถานะ clean ตอนรัน เก็บไฟล์ตัวอย่างแบบไม่แปลง line endings เพื่อรักษา checksums หลัง clone

ชุดทดสอบในเครื่อง Windows/Python 3.12 ผ่าน **20 tests** รวม A* เทียบ Dijkstra บน 20 seeded maps, obstacle/clearance, reproducibility, controller feedback, false arrival, timeout, collision, CLI no-path และ checksums ส่วนผล GitHub Actions ต้องดูสถานะ workflow หลัง push แยกจากผลในเครื่อง

## สิ่งที่ยังเหลือ

P1/P2, lunar DEM/terrain pipeline, radio model, localization/SLAM ที่ตรงขอบเขตวิจัย และ repeated experiments ยังไม่ implement ไม่มีการทดสอบฮาร์ดแวร์ในรอบนี้
