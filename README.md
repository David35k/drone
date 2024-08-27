# DIY Drone
A fast, responsive quadcopter drone that I definetly didn't crash multiple times.

## The Idea 🚀
This epic drone is inspired by modern FPV drones. It was fully designed by me!! Initially I wanted to make a drone that is able to follow me while I am on my bike and take videos. As I made the drone, I realized that just making a working flying drone is kinda hard so I just stuck with making a drone that flies. The object following will be a future project for sure.

## The Design ✏️
I designed my own frame. I did some research and found that 5 inch props are pretty popular and that a 250mm motor to motor distance paired well with them. I used onshape to design the drone. I 3D printed my design using the school 3D printers using PLA plastic. That was probably not the best choice of material but it was conveniently already in the printer lmao. Here is the final design:
<img src="https://github.com/user-attachments/assets/98bee412-733d-4d20-82b4-1bf1f015cf0a" width="500px">

## The Electronics 🔌
The drone is powered by 3 lithium cells. This provides an epic voltage of 12.6V when fully charged. The main flight controller is the ESP32 which can handle this voltage easy peasy. The ESCs (electronic speed controllers) that I am using are the 30A LittleBee-Spring. The motors I am using are a ripoff of the Emax RS2205 motors lmao. The radio transmitter I am using is the FlySky FS-i6 2.4G radio transmitter. It was cheap and came in a set with the FS-ia6 radio receiver. The gyro/accel I am using is the MPU9250 which is perfect for a drone. This operates at 3.3V which is powered by the 3.3V pin on the ESP32.

## The Coding 🤓
The programming is heavily inspired by and based on [this](https://www.youtube.com/watch?v=QvRxxjaLjxg&list=PLeuMA6tJBPKsAfRfFuGrEljpBow5hPVD4) tutorial on YouTube. I highly recommend you watch it if you want to make your own drone. It uses multiple PID control loops to efficiently tilt the drone to the desired angles. This also allows it to balance while standing still (my drone kinda does this for a bit before drifting off lol). My code uses only the angle mode but you could definetly adapt it to use rate mode if you're feeling adventurous.

## The Final Product 😎
Overall, the drone works well. It is very fast and responsive. I still fly it in angle mode because I'm too scared to fly it in rate mode lol. Here is how it looks fully assembled:
<img src="https://github.com/user-attachments/assets/dc8e992a-4b99-47f7-b979-f52a36c17fe6" height="500px">

## Thats cool and all but how do I make my own?
The frame I used is 3D printed but you could definetly use a carbon fiber frame (this is much better than using a silly 3D printed plastic one). Here are the links to the parts I used (cheapest I found on AliExpress):
- [Propellers](https://www.aliexpress.com/item/32677833621.html?spm=a2g0o.order_list.order_list_main.15.55c118021v8Woz)
- [Motors and ESCs](https://www.aliexpress.com/item/1005004476613305.html?spm=a2g0o.order_list.order_list_main.20.55c118021v8Woz)
- [Receiver and Transmitter](https://www.aliexpress.com/item/1005005438498617.html?spm=a2g0o.order_list.order_list_main.25.55c118021v8Woz)

You will also need a microcontroller (I recommend the ESP32) but I already had this so I didn't have to buy it. Same as the MPU9250.
