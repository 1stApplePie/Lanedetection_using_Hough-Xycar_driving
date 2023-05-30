# Lanedetection_using_Hough-Xycar_driving
# 차선인식 오프라인 프로젝트 보고서

# 💡 프로젝트 기획

---

## 프로젝트 목표

해결하려는 문제를 간단히 요약하세요. 이 문제가 해결되어야 하는 이유는 무엇인가요 ?

1. **Offset에 가려진 Lidar**
2. **영상의 밝기 변화**
3. **PID 제어** 
4. **90도 코너 주행**
5. **S자 주행**
6. **직선 주행**
7. **장애물 회피 주행** 

## 프로젝트에 사용한 알고리즘

프로젝트에 사용한 알고리즘과 이 알고리즘을 선택한 이유는 무엇인가요 ?

- Canny Edge
- Hough Transform
- PID
- **LaneKeeping System**

## 이슈사항

### 라이다를 차선으로 인식하는 문제

- 카메라에 라이다가 걸려 **차선으로 인식할 수도 있기 때문에 영상에서 라이다를 지워주는 작업**이 필요하다고 판단했습니다.
    - [**온라인 프로젝트**](https://www.notion.so/5e2c4a2ac4d64333beaf001d337d5585)에서 해결했던 방법을 활용해 오프라인 차선 인식 프로젝트에 적용했습니다.

### 방법 1

- **첫 번째는 warp point를 라이다 위쪽으로 잡아 라이다가 없는 화면의 차선을 보며 주행하는 방법입니다.**
    
    ![스크린샷 2023-05-05 113131.png](README_source/%25EC%258A%25A4%25ED%2581%25AC%25EB%25A6%25B0%25EC%2583%25B7_2023-05-05_113131.png)
    
- 진행할수록 라이다 부근의 차선 데이터에서 다른 차선 데이터보다 양질의 데이터를 얻을 수 있어 두 번째 방법으로 해결하였습니다.

### 방법 2

- **라이다의 mask image를 만들어 기존 이진영상에 subtract 연산을 수행해서 지우는 방법입니다.**

---

- **제공되는 주행 영상을 사용해 mask image를 적용시키는 것을 두 가지 방법으로 테스트했습니다.**
    1. 처음으로 시도했던 방법은 python OpenCV 모듈에 배경을 자동으로 지워주는 모델이 있어 영상에서 움직이지 않는 라이다 부분을 배경으로 생각하고 지워주지 않을까 하는 생각에 코드를 작성하여 결과를 내보았습니다.
        - 사용 모듈: cv2.bgsegm.createBackgroundSubtractorMOG()
            
            ![17.png](README_source/17.png)
            
        - 움직이지 않는 라이다 뿐 아니라 다른 배경까지도 지우는 성능을 보여주어 mask image를 만들기에는 적절하지 않다고 판단했습니다.
    2. **이전 프레임, 현재 프레임을 비교하여 min 값을 추출해 나가며 마스크 만들기**
        - 라이다를 흰 색으로 인식하는 것을 보아 동영상 내에서 움직이지 않는 부분은 항상 흰 색(255)을 갖습니다. 따라서 움직이는 부분들과의 min값을 취해서 움직이지 않는 라이다 부분만을 추출하는 작업을 진행하였습니다.
            
            [2023-05-03 00-50-31.mkv](%E1%84%8E%E1%85%A1%E1%84%89%E1%85%A5%E1%86%AB%E1%84%8B%E1%85%B5%E1%86%AB%E1%84%89%E1%85%B5%E1%86%A8%20%E1%84%8B%E1%85%A9%E1%84%91%E1%85%B3%E1%84%85%E1%85%A1%E1%84%8B%E1%85%B5%E1%86%AB%20%E1%84%91%E1%85%B3%E1%84%85%E1%85%A9%E1%84%8C%E1%85%A6%E1%86%A8%E1%84%90%E1%85%B3%20%E1%84%87%E1%85%A9%E1%84%80%E1%85%A9%E1%84%89%E1%85%A5%209ec47fa365f24d939d2a3583519f9c4e/2023-05-03_00-50-31.mkv)
            

### 결과

- 방법 2는 Python 기준이기 때문에 **C++로 변환 후, bitwise_and 연산을 사용**해서 라이다를 지웠습니다.
    1. 기존 코드는 Python **subtract**를 사용했던 것을 활용 → 255 - mask image로 연산을 대체했었습니다. 
        
        ```cpp
        cv::Mat mask_image = cv::imread("/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/mask_image.png", 0);
        mask_image = 255 - mask_image;
        ```
        
    2. **아예 적용할 mask image를 반전**시키게 되면 빼기 연산이 필요없다는 멘토님의 피드백을 반영해 **반전시킨 mask image를 적용한 뒤 bitwise_and만 사용하는 코드를 사용했습니다.** 
        - 기존 mask image
            
            ![Untitled](README_source/Untitled.png)
            
        - **반전 mask image**
            
            ![Untitled](README_source/Untitled%201.png)
            
        
        ```cpp
        // 영상 전처리, divideLines에서 가져온 선분으로 좌, 우 중점을 return 
        template <typename PREC>
        std::pair<int32_t, int32_t> HoughTransformLaneDetector<PREC>::getLanePosition(const cv::Mat& image)
        {
            cv::Mat mask_image = cv::imread("/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/mask_image.png", 0);
            
            cv::Mat grayImage;
            cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
            
            // cv::Mat non_lidar_image;
            cv::bitwise_and(grayImage, mask_image, grayImage);
        
            cv::Mat canny_image;
            cv::Canny(grayImage, canny_image, mCannyEdgeLowThreshold, mCannyEdgeHighThreshold);
            cv::imshow("canny", canny_image);
        
            cv::Mat ROI = canny_image(cv::Rect(0, mROIStartHeight, mImageWidth, mROIHeight));
            Lines allLines;
            cv::HoughLinesP(ROI, allLines, kHoughRho, kHoughTheta, mHoughThreshold, mHoughMinLineLength, mHoughMaxLineGap);
        
            if (mDebugging)
                image.copyTo(mDebugFrame);
        
            if (allLines.empty())
                return { 0, mImageWidth };
        
            const auto [leftLineIndices, rightLineIndices] = divideLines(allLines);
        
            auto leftPositionX = getLinePositionX(allLines, leftLineIndices, Direction::LEFT);
            auto rightPositionX = getLinePositionX(allLines, rightLineIndices, Direction::RIGHT);
        
            if (mDebugging)
                drawLines(allLines, leftLineIndices, rightLineIndices);
        
            return { leftPositionX, rightPositionX };
        }
        ```
        

### 90도 커브 + S 커브

- **ROI 값, PID Gain 값, Min Speed 값을 조절**해 **커브 주행을 Control** 했습니다.
- 주행 테스트를 통해 원인을 파악하며 여러 값을 수정해야 했는데, 값 하나가 수정되면 다른 값들도 영향을 받아 수정할 때 모든 값을 고려해야 했기 때문에 적정값을 찾는 것에 많은 시간을 소요했습니다.

### 원인 분석

- 출발 지점부터 직선 주행 후 90도 커브까지는 안정적인 주행이 가능했기 때문에 **S자 커브에서 이탈하는 원인**을 찾아 값을 수정하는 것을 목표로 잡았습니다.
    1. S자 커브 진행 전 **90도 커브**가 하나 있는데 **이 커브에서 빠르게 돌지 않고 늦게 돌아 크게 커브를 그리게 되면,** 다음 짧은 직선 주행 후 S자 코스로 진입할 때에도 늦게 돌게 되고 **커브를 크게 그리게 됩니다.** 
        
        ![90도 커브에서 크게 돌고 있는 자이카 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.26.48.png)
        
        90도 커브에서 크게 돌고 있는 자이카 
        
        ![직선 코스에 진입했지만 회전을 크게 했기 때문에 아직 커브를 돌고 있는 모습 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.27.15.png)
        
        직선 코스에 진입했지만 회전을 크게 했기 때문에 아직 커브를 돌고 있는 모습 
        
        ![S자 커브 진입했지만 아직 바퀴가 돌아가지 않은 상태 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.28.01.png)
        
        S자 커브 진입했지만 아직 바퀴가 돌아가지 않은 상태 
        
        ![S자 커브 진입 → 회전 시작이 늦어진 모습 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.28.08.png)
        
        S자 커브 진입 → 회전 시작이 늦어진 모습 
        
        ![S자 코스에서 크게 돌고 있는 자이카 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.28.31.png)
        
        S자 코스에서 크게 돌고 있는 자이카 
        
    2. 커브를 크게 돌면서 카메라에 오른쪽 차선 한개만 보이게 되는데, 이때 옆에 회색 구조물을 오른쪽 차선으로 인식해 **실제 오른쪽 차선 = 왼쪽 차선으로 잘못 감지하면서 차선을 이탈**하게 됩니다. 
        
        ![크게 돌면서 차선 이탈을 하고, 카메라 화면에 한쪽 차선만 보이게 됩니다. ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_6.53.55.png)
        
        크게 돌면서 차선 이탈을 하고, 카메라 화면에 한쪽 차선만 보이게 됩니다. 
        
        ![오른쪽 차선을 왼쪽 차선으로 인식하게 되고, 직선 주행을 할 준비를 하기 위해 바퀴가 오른쪽으로 향하게 됩니다. ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-19_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.29.20.png)
        
        오른쪽 차선을 왼쪽 차선으로 인식하게 되고, 직선 주행을 할 준비를 하기 위해 바퀴가 오른쪽으로 향하게 됩니다. 
        

### 결과

- 90도 커브부터 제대로 주행해야 S자 커브 주행이 가능하기 때문에 90도 커브를 제대로 들어가는 것을 우선적으로 생각하며 **ROI 값, PID Gain 값, Min Speed 값**을 수정했습니다.
    1. ROI 값 수정 == X
        - ROI 영역을 아래로 내리고 올리며 수정 했고, S자 커브 주행에 성공했으나 직선과 이전 커브를 제대로 주행하지 못했기 때문에 가장 안정적이던 365로 값을 돌려놓았습니다.
    2. PID Gain 값 수정 == X
        - 많은 경우의 수를 시도했으나 S자 커브는 물론이고 모든 코스를 불안정하게 주행했기 때문에 PID Gain 값으로 답을 찾는 것은 제외했습니다.
    3. **Min Speed == O** 
        - 곡선 차선을 인지하면 커브를 돌기 위해 속도가 줄어드는데, 이때 **최소 속도값을 크게 줘 빠르게 커브를 주행하도록 하는 것**으로 수정했습니다.
        - **곡선에서 속도가 빨라져 커브를 크게 돌던 문제를 해결했고, 정상적으로 차선에 들어가게 되면서 차선을 이탈하지 않고 주행하는 횟수가 증가했습니다.**
- errorFromMid + 5, 오른쪽으로 치우친 주행 == X
    - S자 커브에서 차선을 잘못 인식하는 것을 원인으로 찾았고, **오른쪽 차선에 붙어서 주행시키며 카메라가 커브를 돌 때 왼쪽 차선을 놓치지 않도록 코드를 수정**했습니다.
    - 차선을 이탈하지 않고 정상 주행을 하는 경우가 너무 적어서 오른쪽으로 치우치는 코드는 추후 제거했습니다.
        
        ```cpp
        template <typename PREC>
        void LaneKeepingSystem<PREC>::run()
        {
            ros::Rate rate(kFrameRate);
            while (ros::ok())
            { 
                ros::spinOnce();
                if (mFrame.empty())
                    continue;
        
                const auto [leftPosisionX, rightPositionX] = mHoughTransformLaneDetector->getLanePosition(mFrame);
                
                mMovingAverage->addSample(static_cast<int32_t>((leftPosisionX + rightPositionX) / 2) + 5);
        
                int32_t estimatedPositionX = static_cast<int32_t>(mMovingAverage->getResult());
        
                int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2) + 5;
                PREC steeringAngle = std::max(static_cast<PREC>(-kXycarSteeringAangleLimit), std::min(static_cast<PREC>(mPID->getControlOutput(errorFromMid)), static_cast<PREC>(kXycarSteeringAangleLimit)));
        
                speedControl(steeringAngle);
                drive(steeringAngle);
        
                if (mDebugging)
                {
                    //std::cout << "lpos: " << leftPosisionX << ", rpos: " << rightPositionX << ", mpos: " << estimatedPositionX << std::endl;
                    mHoughTransformLaneDetector->drawRectangles(leftPosisionX, rightPositionX, estimatedPositionX);
                    //cv::imshow("Debug", mHoughTransformLaneDetector->getDebugFrame());
                    //cv::imshow("Canny", mHoughTransformLaneDetector->getCannyFrame());
                    cv::waitKey(1);
                }
            }
        }
        ```
        

### 직선 구간 감속 문제 + 불안정한 주행

- 직선 구간에서 가속이 되지 않고 좌, 우로 흔들리는 **불안정한 주행을 하며 감속이 되는 현상**을 해결하고자 차선 중앙 범위값과 PID Gain 값을 수정하였습니다.

### 원인 분석

- 직선 주행 시 좌, 우로 흔들리는 현상은 차선 중앙을 찾는 것으로 보였기 때문에 **차선의 중앙 범위 값을 키워주는 것**으로 해결 방안을 생각했습니다.
- 또한 차가 심하게 흔들리는 것은 **오버 슈팅**으로 판단, **PID Gain 값 조절**이 필요하다고 생각했습니다.
    
    [직선 구간 주행.mov](%E1%84%8E%E1%85%A1%E1%84%89%E1%85%A5%E1%86%AB%E1%84%8B%E1%85%B5%E1%86%AB%E1%84%89%E1%85%B5%E1%86%A8%20%E1%84%8B%E1%85%A9%E1%84%91%E1%85%B3%E1%84%85%E1%85%A1%E1%84%8B%E1%85%B5%E1%86%AB%20%E1%84%91%E1%85%B3%E1%84%85%E1%85%A9%E1%84%8C%E1%85%A6%E1%86%A8%E1%84%90%E1%85%B3%20%E1%84%87%E1%85%A9%E1%84%80%E1%85%A9%E1%84%89%E1%85%A5%209ec47fa365f24d939d2a3583519f9c4e/%25E1%2584%258C%25E1%2585%25B5%25E1%2586%25A8%25E1%2584%2589%25E1%2585%25A5%25E1%2586%25AB_%25E1%2584%2580%25E1%2585%25AE%25E1%2584%2580%25E1%2585%25A1%25E1%2586%25AB_%25E1%2584%258C%25E1%2585%25AE%25E1%2584%2592%25E1%2585%25A2%25E1%2586%25BC.mov)
    

### **차선 중앙 범위값 수정**

- 차선을 찾을 때는 감속이 되고, 이때 차선 간 거리가 한쪽으로 치우쳐 있다면 중앙을 찾아 주행할 수 있도록 설계했습니다.
- 문제는 **차선의 중앙이라고 판단하는 적절한 범위 값을 찾는 것**이었습니다.
- 값을 작게주면 곡선 구간에서는 문제가 없지만 직선 구간에서 중앙을 찾기 위해 차가 흔들리면서 감속이 되어 속도가 나질 않았고, 값이 커지면 직선에서는 가속이 되면서 빠르게 주행하지만 곡선 주행을 제대로 하지 못하고 차선을 이탈했습니다.
- 계속해서 주행 테스트를 하며 적절한 값을 찾는 것 밖에는 방법이 없어서, **10 - 100 사이의 값을 주면서 직선과 곡선을 안정적으로 주행하는 값을 찾았습니다.**
    
    ```cpp
    template <typename PREC>
    PREC PIDController<PREC>::getControlOutput(int32_t errorFromMid)
    {
        // int32_t errorFromMid = estimatedPositionX - static_cast<int32_t>(mFrame.cols / 2);
        PREC castError = static_cast<PREC>(errorFromMid);
    
        float straight_mPropertionalGain;
    
        mDifferentialGainError = castError - mProportionalGainError;
        mProportionalGainError = castError; 
        mIntegralGainError += castError;
        if (std::abs(castError) < 66)
        {   
            straight_mPropertionalGain = mProportionalGain / 2;
            //return straight_mPropertionalGain * mProportionalGainError + mDifferentialGain * mDifferentialGainError;
            return straight_mPropertionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError;
        }
    
        mIntegralGainError = std::max(mIntegralGainError,static_cast<PREC>(500.0f));
    
        return mProportionalGain * mProportionalGainError + mIntegralGain * mIntegralGainError + mDifferentialGain * mDifferentialGainError;
    }
    ```
    

### **PID 제어 → 적정 PID Gain 값 찾기**

- 적절한 차선 중앙 범위값을 찾았지만 한번도 이탈 없이 주행하는 것은 아니였으므로 **확률을 높이기 위해 PID Gain값을 수정하면서 안정된 값을 찾았습니다.**
    
    ![스크린샷 2023-05-14 오후 11.43.08.png](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-14_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_11.43.08.png)
    
    ![스크린샷 2023-05-14 오후 11.43.30.png](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-14_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_11.43.30.png)
    
    ![최종적으로 설정한 값은 빨간 네모 박스로 표시했습니다. ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-14_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_11.43.47.png)
    
    최종적으로 설정한 값은 빨간 네모 박스로 표시했습니다. 
    

### 라이다를 이용한 장애물 회피 주행

- 라이다 회피 주행은 차선 인식 주행을 완벽하게 진행해야 하는 선행 작업이 필요했습니다.
- 일정 수준의 완성도를 갖춘 주행 코드를 기반으로 라이다 주행을 하려고 팀의 두 번째 자이카에 코드를 담아 주행을 해 보았지만, PID제어 특성상 해당 자이카에 맞는 PID값을 먼저 찾아내야 라이다 회피 주행이 시작 가능했습니다.
- 책상 위 두 번째 자이카를 고정하고 라이다 인식 후 일정 시간 회피 주행이 되는 것을 확인하고 주행용 자이카에 코드를 옮겨 진행하였으나, 주행용 자이카에서는 해당 코드가 작동하지 않는 현상이 있었습니다.
- 해당 현상에 대한 원인 파악과 수정을 거치기에는 PID값 수정과 다른 변수 조정이 완벽하게 끝나지 않았기에 실제 주행에서는 적용하지 못하고 다른 자이카에서 작동하였던 코드를 남겨두었습니다.
    
    ```cpp
    template <typename PREC>
    void LaneKeepingSystem<PREC>::lidarDetecting(float leftMinValue, float rightMinValue)
    {
        xycar_msgs::xycar_motor motorMessage;
        
    		/**
    		* 왼쪽 Lidar Detecting range가 40cm보다 작을 때
    		* 차량이 1초간 우회전 후 1.5초간 좌회전하여 기존 차선을 찾아오도록 함
    		*/
        if(leftMinValue<0.4){
            ros::Time start_time = ros::Time::now();
            ros::Duration turn_right(1);
            ros::Duration turn_left(1.5);
    
            while(ros::Time::now() - start_time<turn_right){
                motorMessage.angle = 50;
                motorMessage.speed = 10;
                mPublisher.publish(motorMessage);
            }
            while(ros::Time::now() - start_time<turn_right+turn_left){
                motorMessage.angle = -50;
                motorMessage.speed = 10;
                mPublisher.publish(motorMessage);
            }
        }
    
    		/**
    		* 오른쪽 Lidar Detecting range가 40cm보다 작을 때
    		* 차량이 1초간 좌회전 후 1.5초간 우회전하여 기존 차선을 찾아오도록 함
    		*/
        if(rightMinValue<0.4){
            ros::Time start_time = ros::Time::now();
            ros::Duration turn_right(1);
            ros::Duration turn_left(1.5);
    
            while(ros::Time::now() - start_time<turn_right){
                motorMessage.angle = -50;
                motorMessage.speed = 10;
                mPublisher.publish(motorMessage);
            }
            while(ros::Time::now() - start_time<turn_right+turn_left){
                motorMessage.angle = 50;
                motorMessage.speed = 10;
                mPublisher.publish(motorMessage);
            }
        }
    
    }
    ```
    

## 협업 과정

- 저희 팀은 게더를 사용해 회의를 진행했습니다.
    
    ![2023-5-5 오프라인 프로젝트 첫 회의, 8일까지 공부해야할 코드와 알고리즘에 대해 회의하는 시간을 가졌습니다. ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-05_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_5.22.09.png)
    
    2023-5-5 오프라인 프로젝트 첫 회의, 8일까지 공부해야할 코드와 알고리즘에 대해 회의하는 시간을 가졌습니다. 
    
    ![2023-5-10 민지님과 코드 보충 시간 ](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-10_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%2592%25E1%2585%25AE_11.41.30.png)
    
    2023-5-10 민지님과 코드 보충 시간 
    

# 🏁 프로젝트 결과

---

## 결과

![스크린샷 2023-05-14 오전 12.28.44.png](README_source/%25E1%2584%2589%25E1%2585%25B3%25E1%2584%258F%25E1%2585%25B3%25E1%2584%2585%25E1%2585%25B5%25E1%2586%25AB%25E1%2584%2589%25E1%2585%25A3%25E1%2586%25BA_2023-05-14_%25E1%2584%258B%25E1%2585%25A9%25E1%2584%258C%25E1%2585%25A5%25E1%2586%25AB_12.28.44.png)

[주행 테스트 영상](%E1%84%8E%E1%85%A1%E1%84%89%E1%85%A5%E1%86%AB%E1%84%8B%E1%85%B5%E1%86%AB%E1%84%89%E1%85%B5%E1%86%A8%20%E1%84%8B%E1%85%A9%E1%84%91%E1%85%B3%E1%84%85%E1%85%A1%E1%84%8B%E1%85%B5%E1%86%AB%20%E1%84%91%E1%85%B3%E1%84%85%E1%85%A9%E1%84%8C%E1%85%A6%E1%86%A8%E1%84%90%E1%85%B3%20%E1%84%87%E1%85%A9%E1%84%80%E1%85%A9%E1%84%89%E1%85%A5%209ec47fa365f24d939d2a3583519f9c4e/_talkv_wtikrtAex4_tC0lb4J2Fv4oMs97htIeAk_talkv_high.mov)

주행 테스트 영상            

---

# 📢 향후 작업 및 소감 (선택)

---

프로젝트가 끝난 이후에도 더 작업을 진행한다면 어떤 부분들을 할 예정인가요 ?
