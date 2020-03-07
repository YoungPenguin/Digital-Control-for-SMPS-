# Digital Control for SMPS in CPT Charging System


Digital Control for SMPS in CPT Charging System
Done with embedded C on a C2000 TMS320F28027F DSP MCU.

# Transfer function
The Control-to-Output transfer function is found by modeling:

![first equation](https://latex.codecogs.com/gif.latex?G_%7Bvd%7D%3D%5Cfrac%7BV%20%5Cleft%28R-%5Cfrac%7BDL%7D%7BD%27%5E2%7Ds%20%5Cright%29%7D%7BLRCs%5E2&plus;Ls&plus;RD%27%5E2%7D)

This is as expected the same as a buck-boost as the flyback is just an isolated version of this. With paracitic parameters the transfer function becomes:

![first equation](https://latex.codecogs.com/gif.latex?G_%7Bvd%7D%3D%5Cfrac%7BV_%7Bg%7D&plus;V-%5Cfrac%7BR_L&plus;Ls%7D%7BD%27%7DI_L%7D%7BD%27%20%5Cleft%28%5Cleft%28%5Cfrac%7BLC%7D%7BD%27%5E2%7D%20%5Cright%29s%5E2&plus;%20%5Cleft%28%5Cfrac%7BR_LC&plus;L/R%7D%7BD%27%5E2%7D%20%5Cright%29s&plus;%20%5Cleft%281&plus;%5Cfrac%7BR_L/R%7D%7BD%27%5E2%7D%20%5Cright%29%20%5Cright%29%7D)

# Implementation 
Implementet PI controller with ADC at ADCINA0 and out at PWM4A (GPIO6) @ 150 kHz - TBPRD 330

ADCINA4 AA4 (j1-6) -
GPIO6 P6 (j2-13)-

The implementation of the PI algorithm is done in discrete time as follows: 

![first equation](http://latex.codecogs.com/gif.latex?%5Cfrac%7Bu%28s%29%7D%7Be%28s%29%7D%3DK_%7Bp%7D%5Cleft%20%28%201&plus;%5Cfrac%7B1%7D%7BT_%7Bi%7Ds%7D%20%5Cright%20%29)

![secound equation](http://latex.codecogs.com/gif.latex?u%28s%29%3DK_%7Bp%7D%5Cleft%20%5B%20e%28t%29&plus;%5Cfrac%7B1%7D%7BT_%7Bi%7D%7D%5Cint_%7B0%7D%5E%7Bt%7De%28t%29%20dt%20%5Cright%20%5D)

![](http://latex.codecogs.com/gif.latex?u%28k%29%3DK_%7Bp%7D%5Cleft%20%5B%20e%28k%29%20%5Cfrac%7B1%7D%7BT_%7Bi%7D%20%7D%20%5Csum_%7Bn%3D0%7D%5E%7Bk%7D%20e%28n%29%20T_%7Bs%7D%5Cright%20%5D)

![](http://latex.codecogs.com/gif.latex?u%28k-1%29%3DK_%7Bp%7D%5Cleft%20%5B%20e%28k-1%29%20%5Cfrac%7B1%7D%7BT_%7Bi%7D%20%7D%20%5Csum_%7Bn%3D0%7D%5E%7Bk-1%7D%20e%28n%29%20T_%7Bs%7D%5Cright%20%5D)

![](http://latex.codecogs.com/gif.latex?%5CDelta%20u%28k%29%3Du%28k%29-u%28k-1%29%3DK_%7Bp%7D%5Cleft%20%5B%20e%28k%29-e%28k-1%29%20%5Cright%20%5D%20&plus;%20K_%7Bp%7D%20%5Cfrac%7BT_%7Bs%7D%7D%7BT_%7Bi%7D%7De%28k%29)

![](http://latex.codecogs.com/gif.latex?u%28k%29%3Du%28k-1%29&plus;%5CDelta%20u%28k%29)

![](http://latex.codecogs.com/gif.latex?%3Du%28k-1%29&plus;K_%7Bp%7D%281&plus;%5Cfrac%7BT_%7Bs%7D%7D%7BT_%7Bi%7D%7D%29e%28k%29&plus;K_%7Bp%7De%28k-1%29)

The transfer function is found by modeling and regulation is done with matlab and simulink

