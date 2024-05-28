# Drone
a very **epic** drone, my 2024 robotics project

> may 2024 update: it flies!! (ﾉ◕ヮ◕)ﾉ*:･ﾟ✧

> note: readme still not finished 🤦
> note: insert cool image of the actual thing lmao

## The Idea 🚀
This epic drone is inspired by modern FPV drones. It was fully designed by me!! Initially I wanted to make a drone that is able to follow me while I am on my bike and take videos. As I made the drone, I realized that just making a working flying drone is kinda hard so I just stuck with making a drone that flies. The object following will be a future project for sure.

## The Design ✏️
I designed my own frame. I did some research and found that 5 inch props are pretty popular and that a 250mm motor to motor distance paired well with them. I used onshape to design the drone. I 3D printed my design using the school 3D printers using PLA plastic. That was probably not the best choice of material but it was conveniently already in the printer lmao. Here is the final design:
<img src="https://github.com/David35k/drone/assets/62733874/9d4b14c1-e09f-4733-964d-f5116073a953" width="500" alt="image of epic design">
<img src="https://github.com/David35k/drone/assets/62733874/0f39b3e3-470a-4b7e-9865-ca167f9c48b8" width="500" alt="another image of epic design">

## The Electronics
two words: ultra sketchy
The whole shebang is powered by a good ol' 3 cell lipo battery. Mounting is _sketchy_ to say the least (its just two pieces of velcro 😭). This provides an epic voltage of 12.6V when fully charged. The main flight controller is the ESP32 which can handle this voltage easy peasy. The ESCs (electronic speed controllers) that I am using are the 30A LittleBee-Spring. The motors I am using are a ripoff of the Emax RS2205 motors lmao. The radio transmitter I am using is the FlySky FS-i6 2.4G radio transmitter. It was cheap and came in a set with the FS-ia6 radio receiver. This receiver is great but it takes a max voltage of 6V which is kinda half of what the battery supplies. I forgot about this and almost fried it. It is for this reason that I had to add the voltage converter which incrased weight slightly but the drone handled it like a champ 😎. With the added voltage converted I can add other 5V components such as an FPV camera which I hope to do in the future. The gyro/accel I am using is the MPU9250 which is perfect for a drone. This operates at 3.3V which is coming from a pin on the ESP32, very epic.

> note: insert cool image of the actual thing again lawl

## The Coding
four words: actually not that bad 😼
