# Lunar Rover Autonomy

โครงงาน Senior Project สำหรับศึกษาการนำทางรถสำรวจดวงจันทร์ด้วยการจำลอง โดยเปรียบเทียบการประมาณตำแหน่งและการวางแผนเส้นทางที่คำนึงถึงภูมิประเทศกับการสื่อสาร

**สถานะ 24 ตุลาคม 2026:** เริ่มต้นซอฟต์แวร์ระยะที่ 1 มี P0 และรถจำลองแบบป้อนกลับบนสนามสังเคราะห์ ยังไม่มีข้อมูลภูมิประเทศดวงจันทร์จริง ระบบ SLAM หรือการควบคุมรถจริง

## เริ่มใช้งาน

ต้องมี Python 3.12 ขึ้นไป รันคำสั่งจากโฟลเดอร์ repo ตัวอย่างสำหรับ Windows PowerShell:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m pip install -e .
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
.\.venv\Scripts\python.exe -m lunar_rover --scenario configs/scenarios/synthetic_obstacles.json --localization ideal --seed 42 --output outputs/ideal-seed42 --animate
```

macOS/Linux ใช้ `.venv/bin/python` แทน `.venv\Scripts\python.exe` หรือ activate environment แล้วเรียก `python` โดยตรง

`requirements.txt` บันทึกเวอร์ชันที่ใช้ทดสอบ ตัวแพ็กเกจระบุช่วงเวอร์ชันที่รองรับใน `pyproject.toml` การรันครั้งใหม่ต้องเลือก output directory ใหม่เพื่อไม่ทับผลเดิม

## สิ่งที่ทำงานแล้ว

- อ่านสนามสังเคราะห์จาก JSON พร้อมหน่วย พิกัด และ seed
- **P0:** A* ระยะทางสั้นที่สุดบนกริด 8 ทิศทาง โดยใช้ hard obstacles ร่วมกับรัศมีตัวรถและระยะเผื่อ ห้ามตัดมุมสิ่งกีดขวาง
- รถจำลองบนระนาบรับคำสั่งความเร็วเชิงเส้นและอัตราหมุน ตัวควบคุมรับเฉพาะตำแหน่งประมาณ
- ตรวจการกวาดผ่านสิ่งกีดขวางระหว่างตำแหน่ง ไม่ตรวจเฉพาะปลายก้าว
- ส่งออกเส้นทางที่วางไว้ ตำแหน่งจริง/ประมาณ กราฟ RMSE ข้อมูล CSV และ run manifest พร้อม SHA-256
- ทดสอบความถูกต้องของ A* เทียบ Dijkstra รวมถึงทางตัน ระยะเผื่อ ตัวควบคุม และการรันซ้ำ

## โมเดลตำแหน่งสำหรับตรวจต้นแบบ

| CLI mode | ความหมาย | ข้อจำกัด |
|---|---|---|
| `ideal` | อินทิเกรต odometry ที่ไม่มี noise/bias จากคำสั่งการเคลื่อนที่ | กรณีตรวจซอฟต์แวร์ ไม่ใช่สมรรถนะเซนเซอร์จริง |
| `odometry` | เพิ่ม scale bias, yaw-rate bias และ Gaussian noise | เป็นแบบจำลองความคลาดเคลื่อนจากคำสั่ง ยังไม่มี encoder/IMU จริง |
| `fixes` | odometry แบบเดียวกัน พร้อมตำแหน่งอ้างอิงสังเคราะห์เป็นช่วง ๆ | ใช้ Gaussian noise และ gain คงที่ ไม่ใช่ GNSS receiver model หรือ SLAM |

ทุกโหมดเริ่มด้วยตำแหน่งและทิศที่ทราบแน่นอน แผนที่สิ่งกีดขวางทราบล่วงหน้า โมเดลนี้ยังไม่รวม wheel slip, dynamics หรือ kinematics ของ 6WD/4WS และไม่ได้เปลี่ยนแบบรถจริงไปเป็น skid-steer

ตัวอย่างตรวจความคลาดเคลื่อน:

```powershell
.\.venv\Scripts\python.exe -m lunar_rover --scenario configs/scenarios/synthetic_obstacles.json --localization odometry --seed 42 --output outputs/odometry-seed42
.\.venv\Scripts\python.exe -m lunar_rover --scenario configs/scenarios/synthetic_obstacles.json --localization fixes --seed 42 --output outputs/fixes-seed42
```

Exit code `0` = ถึงเป้าหมาย, `2` = ผลการทดลองไม่สำเร็จ เช่น ชน/หมดเวลา/ไม่มีทาง, `1` = input หรือการรันมีข้อผิดพลาด การชนจะยุติการเคลื่อนที่ก่อนก้าวที่ไม่ปลอดภัย และบันทึก `status: collision`

## เปิดดูผล

ใน output directory แต่ละรอบจะมี:

| ไฟล์ | เนื้อหา |
|---|---|
| `overview.png`, `overview.svg` | แผนที่ เส้นทางที่วางไว้/วิ่งจริง/ตำแหน่งประมาณ และกราฟ position error |
| `trajectory.gif` | ภาพเคลื่อนไหวเมื่อระบุ `--animate` ย่อเวลาเพื่อดูภาพรวม |
| `trajectory.csv` | ตำแหน่งจริงและประมาณทุกก้าว พร้อมคำสั่งและความคลาดเคลื่อน |
| `planned_route.csv` | เซลล์และพิกัดเส้นทาง P0 |
| `metrics.json` | สถานะ RMSE ระยะทาง เวลา และระยะห่างเป้าหมาย |
| `scenario.json` | สำเนา input ของรอบนั้น |
| `manifest.json` | seed, units, source hash, Git revision/dirty state, dependencies และ output checksums |

ถ้าไม่มีเส้นทาง จะมีเฉพาะ input, metrics และ manifest โดยไม่สร้าง trajectory ปลอม

RMSE คือ `sqrt(mean((x_est-x_true)^2 + (y_est-y_true)^2))` บน timestamp เดียวกัน เส้นทางที่วางไว้กับระยะที่วิ่งจริงรายงานแยกกัน รถอาจหยุดก่อนศูนย์กลางเป้าหมายตาม tolerance และตัวควบคุมอาจเลี้ยวเป็นโค้ง จึงไม่ควรตีความระยะวิ่งที่สั้นกว่าเป็นการปรับปรุง planner

ผลจาก seed เดียวเป็นเพียงการตรวจต้นแบบ ดูตัวอย่างที่ [examples/seed42](examples/seed42) และ [บันทึกผลรอบแรก](docs/baseline-results.md) ห้ามสรุปว่าโหมดหนึ่งดีกว่าอีกโหมดทางสถิติจากตัวอย่างนี้

## โครงสร้างและงานถัดไป

```text
configs/scenarios/       สนามและพารามิเตอร์ JSON
src/lunar_rover/         grid, P0, simulation, logging, plots และ CLI
tests/                  automated tests
docs/                   แผนการทดลอง การตัดสินใจ และผลตรวจ
examples/seed42/         ผลตัวอย่างสังเคราะห์ที่เก็บไว้ตรวจสอบ
outputs/                ผลที่ผู้ใช้สร้างเอง ไม่ติดตามใน Git
```

อ่าน [แผนการทดลอง](docs/experiment-plan.md) และ [การตัดสินใจ/สมมติฐาน](docs/decisions.md) ก่อนเพิ่ม P1/P2 หรือระบบระบุตำแหน่ง ขอบเขตถัดไปคือกำหนดการเปรียบเทียบตำแหน่งให้ชัด เพิ่มภูมิประเทศ/การสื่อสาร แล้วทดลองซ้ำหลายฉากและหลาย seed

งบซื้อฮาร์ดแวร์ที่คุยไว้คือ **6,000 บาท** โดยทีมมี Raspberry Pi 5, ESP32 และ ABS อยู่แล้ว งบนี้ยังไม่ใช่ BOM ที่ยืนยันราคา/แรงบิด/ระบบไฟ และไม่เป็นเงื่อนไขเลือกพารามิเตอร์จำลองรอบนี้

## License

MIT ตาม [LICENSE](LICENSE) โค้ดต้นแบบรอบนี้เขียนขึ้นใน repo นี้ ยังไม่ได้นำ CAD หรือโค้ดจาก JPL/ExoMy มาใช้ การอ้างอิงแนวคิดรถไม่ได้หมายถึง NASA/JPL รับรองโครงงาน
