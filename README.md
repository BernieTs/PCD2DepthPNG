# PCD2DepthPNG
point cloud data(.pcd) transfer to 2D depth image(.PNG)


## 目的
此程式參考**coldnew's blog**中的[將 pcd 檔轉換成深度圖與 RGB 圖檔]文章。將有序的point cloud data轉換成深度影像並儲存成PNG檔。   
但有些3D取像感測器(keyence, gocator...)取得的Z值範圍通常有正值或負值。通常有一個虛擬基準零點，量測值會落在正負量測範圍內(fig. 1)。  
所以程式這邊改寫，加入shift的概念，避免輸出的深度影像會有全黑的情況。


![measurement distance](/img/LJV7200.png "量測範圍示意")    
*Fig 1. 量測範圍示意  (1)虛擬零點：離感測頭200mm距離   (2)量測距離：離虛擬零點正負50mm*  

[將 pcd 檔轉換成深度圖與 RGB 圖檔]:https://coldnew.github.io/56ebd889/ 

## 使用
開啟**PCD2DepthPNG.exe**檔案，並拖曳或輸入**有序的**pcd檔案位置(fig. 2)，接著程式將於同資料夾下生成一個檔名相同的png檔案。

>注意：如輸入附檔名不是.pcd，則程式會一直要求重新輸入。 

>注意：輸入的路徑最好不要有空格。

![console](/img/console.png "程式畫面")    
*Fig 2. 程式操作畫面*   

![3維點雲資料](/img/test_pcd_20200630_1.pcd.png "3維點雲資料")    
*Fig 3. 3維點雲資料*  

![轉換後之深度影像圖](/img/test_pcd_20200630_1.png "轉換後之深度影像圖")    
*Fig 4. 經過程式轉換後之深度影像圖*  


