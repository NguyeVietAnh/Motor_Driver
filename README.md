# Motor_Driver
Hi every one! This is my project using STM32F103 and LMD18200 to control DC motor with close-loop feedback.
![](http://f10.photo.talk.zdn.vn/2456219225407768173/653f6230c79527cb7e84.jpg)
# Altium Design
![untitled](https://user-images.githubusercontent.com/23720583/48367631-e3e1f080-e6e3-11e8-95b4-366bb11d108c.png)

![untitled](https://user-images.githubusercontent.com/23720583/48367773-4affa500-e6e4-11e8-93fb-d44bc452375a.png)

# Diary:
- Day 13/11/2018: Writing the test module for Timer and Direction button. Timer is working, also button but wrong pin of control LMD182001. UARST running on UARST2 with PIN PA2, PA3. (test_idea branch)
- IC MAX232 too hot when running, so I replace the new one and it run normally, thanks god!

- Day 15/11/2018: Implement NodeMcu module in project, Wifi testing ok, Motor and project can work through Wifi 
