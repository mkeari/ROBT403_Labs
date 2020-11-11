After some fine-tuning the 512-256-128 architecture has been chosen, which showed a significant improvement from not-tuned model:

<p align="center">
<img src="/lab_week10/images/512-256-128_5000_10_100_train.jpg" alt="512-256-128_5000_10_100_train" width="900"/>
</p>
<p align="center">
<img src="/lab_week10/images/512-256-128_5000_10_100_test.jpg" alt="512-256-128_5000_10_100_test" width="900"/>
</p>
Average error of 5.9%

Then, the parameters of data size, batch size, N of epochs were tuned:

Data size increased to 8000;
Epoch number increased to 15;
Mini-batch size increased to 200;

<p align="center">
<img src="/lab_week10/images/512-256-128_8000_15_200_train.jpg" alt="512-256-128_8000_15_200_train" width="900"/>
</p>
<p align="center">
<img src="/lab_week10/images/512-256-128_8000_15_200_test.jpg" alt="512-256-128_8000_15_200_test" width="900"/>
</p>
Average error of 3.9%
