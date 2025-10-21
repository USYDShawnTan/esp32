# Audio Assets

Place pre-generated WAV clips in this directory so the ESP32 can download and play them.

- Files are served at `http://<backend-host>:<port>/data/<filename>`.
- The sample dashboard (`/static/index.html`) expects optional clips named `alert_fall_detected.wav` and `status_all_clear.wav`, but you can upload any other WAV files as needed.

This placeholder file keeps the folder under version control; remove it if you prefer a clean directory.***
