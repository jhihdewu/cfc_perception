# Models
## yolo11n-seg.pt:
* 預訓練的yolo v11分割模型，參數量為2.6M。

## raw.pt:
* 在yolo11n-seg上進行微調，使用的標籤直接來自 Grounded-SAM-2 的輸出。

## processed.pt:
* 在yolo11n-seg上進行微調，使用的標籤是經過處理的 Grounded-SAM-2 標籤。
* 所謂的處理，是指我利用 CVAT 對 Grounded-SAM-2 的標籤和資料進行了以下操作：
    * 移除重複的偵測
    * 增加遺漏的標籤
    * 刪除錯誤的標籤