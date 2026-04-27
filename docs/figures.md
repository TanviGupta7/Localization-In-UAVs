# 📊 Figures & Image Documentation

## UAV Localization using ORB-SLAM3 + VIO

This document describes all figures used in this project. Each figure is designed to help understand the UAV localization pipeline, system architecture, datasets, and results.

---

# 🧠 Figure 1: Visual SLAM Pipeline

<img src="https://images.openai.com/static-rsc-4/9GTcTaYnYwDHv0I8BBBkhu3Ya_kvWULXF_vyoNoDBwM6-IFuNB71_rIx6ANNIpXBxZq8CVfvQ9m12wqjYZqdFLom6GTIBxzS0vOPMj7otN5UsJSfqxZbSmPnZrv3DQtJ5IN7TQ2sT1gl9VSlGoz2n0XBQtwBw0RnUDObgJ87o7j6_dScDKzg0WWrOpIUvRcg?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/7FISXYD7WEPTFaBZsGCgjfSv3BdOaQVtBim-YTFFzQeVCL35ErEiaoqqPIFPt-413Q8ZOIc1wHJ0gAEuwiC8vwHb4RPhf2R792PkLCvumpmoMXzM58GHrmfMPx98rSGwpec61l4Wz8Pu-AKUiAavlLoCgDGun2L8asjcaBbORLB1E8HgQ1s9R_CWs1ZgUMlp?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/joPNnp28JOL4S2-FqicKRrjb9fAXqQ1fkD3_HhX36GFTywWkKwjybtm8NF902IDMTH0j916OdPmSzLqtAF1nKi1t5FE5KHVAJ4icuYGkFlhfaN6NoZkJubkm_IqhaTTPqgLWzeUBwErTxzsYXiXaGUGLTI1WPxY5n1w23WEniDUELpuiW6gXaWekQgR4xfSP?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/BndePkWDJ80GkQneLThLS3Ey4_lHt4VrY5onEDrPXVCy3kBVSnHRzlybeTkXv6uabNO_ukeADumrG4hIaMrkxMbjGaJry4JKyc3PXbeu-04zOR6jdBMvB214B1hpwKZ9WUIgx_ywDUmFGATqLsmd0LLL7JGX9z_LFWY0UJE5MqRmplLhnV6wD4YrtHWfr3Wo?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/AjrA2SdwTeZV0-fQt995Chf5h52KQKj6KIYclAj9JH3xnQ8vI4Pu3LGynmA9XA2ILx6JjmpYIwpyAQNIsm8S9a9AlQcqwXl_Y6T_o0PJBm1mkyx6Dq16p4iwZNkKuw7g5WVfj01eGZn1V_3r7pUISC33DsNK_GgARIi-vPOQ_CMwxJ0hP3Hy8z-oGUpoaWxj?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/0q8rF3YYb_pLN9Sc7LKc4RWKEMnoXpFxQoXx2AR7SY0XtkGpUxDrSRcOTif8zSHGYCdcyy4CXIiGQj4kQY292B9gRBQXKkB0ZnLjJSyNC5zHmYzt_k2iR_hiKOgVlp5XvfO56fRUQaL1f2D3DgaZGz6DH3g-Sl_Pmy9RMqz1q6ysqCpnSdwPNugLEp8U1YOz?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/jtDLoE97sUZL93et1VpoYzvpxgp8uZrqODm_kC-ufSE-h1Qzre30yClX_boIjOgd4n_t5qvFaxi-188TBI3EYS_zmpWRrmESeE4TUp2BTk2qVD8BeiSgy_4wkEhKjpYECoHiwZld2N4kWC3AtFgMpuGAgk1Uy5L-DUXBDRX-yS76OhPj4zrXC3L1h6Gn2ZQx?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/q9Ls-pEvDrdnmIctXz5Ga9JbIWJcWc6t-xV2BRFBCRYIqgGOS1jlxgPFilO9wVMoq-hIVZbW199LbnbAKl8I-RfUHhx4XuTIqR_PR0_b4qBDYsuUvfVxkAhi5B2y7i7vUEliPr1glUoAkT9oEXbnaZzobO2JuNuUnR1hgbA54My_P-rD3PTgvcycOhzXWIRl?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/tpc9oXSInt3ZNIwPs8deYgzWWLbENJQxwkrmqVzhuHB6-E9hb1qtVSOjRP1lwm33UjqNuesJAF7L5NVb0TbS9oNHPMJ0nqOChThuergLEDLtFlhf_CVbyO80ESQ9MyBKlyffqpLfjNMM6bEnTZwjFQISGa03D43VgwbTE_pBv_tFmY_aBIHJCuF9WQ1sN_69?purpose=fullsize">

### 📌 Description

This figure explains how SLAM (Simultaneous Localization and Mapping) works.

### 🔍 Key Components

* Feature Detection (ORB features)
* Feature Matching between frames
* Pose Estimation
* Map Generation
* Loop Closure (error correction)

### 💡 Importance

This is the **core process** that allows a drone to:

* Understand its surroundings
* Track its position continuously

---

# 🧭 Figure 2: Visual-Inertial Odometry (VIO)

<img src="https://images.openai.com/static-rsc-4/pVSjaBrw8MhUfwxOUfzuNJU7Shgg4vCKa6xrABsAww_-HKP3O4R7nQordLMsk0Nj6O__m_jUOJRX8tb7-xrZ-WoGTzp32bKxbRFf2mNJbUJEfnaDHgQ95q4XPerJ5TTgjnGBboHsKffx_nUxhCTCtf19GJNHN-YRebB0yu-Gfa89Mvi2Bei_5BE33cu3ni2L?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/vcMsmPPZYyOaqKZRPnbMG2ShzPOEkfY7BEFIwU7Rzfb8gNR2FxEtdgwNYonIxKFrSfGrjnpYsSkdK5-OxD_OX8muiA3PzZhhL3OFi8rAB3M6DTDSTzAu6H8BNqcdQvrbkxHNi9Es5IU9knfYvWDOyj0s4JYM2oFrVYQsHAD81F4UFbRPEA-AWghRXlSuPVCi?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/y1yZjeTPfoLERhJfZpgQ1DLugoBXISHV5uotyAwjDp61MOUkW2aU7oNNX6HvPBIprg2q682yymd7DiNGcEZM_YjB6PkEzsUKFvg9VPzHXMR9G4c9R50gj4pCkwK9ifZkgvH_TmZqsHIaybZCsouWcKY_69hH3G3f4yZJtqMAE1zM7wG5pAc90BfvyQeggjnp?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/9Z28HA0sfDvwF3ggsWV9hJZ7vhEMJyPUMvkbHxfhMzm5t-_esttcHbpP0Tt7kJKKikFPVeMMsr98ORuGLSQe_LF2FQYX6ki-74yY8tMhx69CDrmlycnBqf80s2M7xKspiqYPmXpHmEPHjl6nCXSIm9bwXIyOHSkGA4DSu3Xuw6JptYIFCM4h-hnp0lDX5ltX?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/tgXm2oXzMdgTpptWG5FPjIZ1DEwvc6Y0W5vtCEZbnS_z45zsqovZTFEJ8cri2K7x1s6UthgPWjsUGg2YPvADLyn-k3c5D9MT1aqt4HFsfUmQmsUdQsmh7UaFDyP9Mi6N194qzijQVcwb1CZKjtmgbNEfUnI_Zrby2kkN7LC2_J9_oIXXv9Agx0FRZ72ks12I?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/lGtEfBD9bcPEqetAgFHbvop0q8rZndhQdrFyqMwkX6WdraMgPbrHrksxBH8Qdee_6MLltXSXpLUttxMBePauZlO7AutPaG2ZJ_Xa8Aq4Vg-EOdQ0_cWhUDrjgEiP4TGBIQcZYO-XJr2eIBOo9k11Wv4ZR6hKMgZ9wizkjETy-lCwHELJZXwpcsyfntofi2Pf?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/ccpFXLrd1d32mRAwMRtrGAH9_Zm9xJd0pXmRKLuqLnN-EZ-UERIwUiQcBfDPNzN5spF9DGVsZX6FFHHpsaICSvQuNSxVMk9AmPRttY4RRZkRSe0GZJlU6IphjmPuSnkmSH5kzyiacJM8hNqF1UwgHFoyMOOMoP-HXHT_AjOTdnU_bxNHCVX7P5_MqKWlhoqf?purpose=fullsize">

### 📌 Description

This figure shows how camera and IMU data are combined.

### 🔍 Key Components

* Camera → Visual features
* IMU → Motion (acceleration + rotation)
* Sensor fusion → Accurate pose estimation

### 💡 Importance

* IMU helps during fast motion
* Camera corrects IMU drift
* Together → stable localization

---

# 🏗️ Figure 3: UAV Localization Architecture

<img src="https://images.openai.com/static-rsc-4/_MWYv6QpVZx_ZWuw4e3plct9Dd1TRA4RXyRkn4D_AoPHw47P2v1C81zN5OmlJE3OaN2tK0eIYelQwr82REfa05TRhzZOaJhRkXJQbS7PmDYvBztRixUBpLijrmePhY8-b8p_3e0mIxOasfsSvJJp37xMcXvxAaXVqObofUhedw2pkP1_iCVJMfMlEygIgDGv?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/CAE_W_G6MXUoznrv6AlKtEcPdvAfDwI5kzdd5cpnMwNUSk4ciEfQYN96H4ICY6ukViOHI9uV5PrRppCQMA_efwarwMXiRYnwVx-z-hllt_zZZxESZLRzLZ5hbmDLrlb9F5IjFAidemetWFQY24FsaNRTHqV2RnnAvXj5taPXXMgJQSXqogH8hi3M2v9VN-OE?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/7FISXYD7WEPTFaBZsGCgjfSv3BdOaQVtBim-YTFFzQeVCL35ErEiaoqqPIFPt-413Q8ZOIc1wHJ0gAEuwiC8vwHb4RPhf2R792PkLCvumpmoMXzM58GHrmfMPx98rSGwpec61l4Wz8Pu-AKUiAavlLoCgDGun2L8asjcaBbORLB1E8HgQ1s9R_CWs1ZgUMlp?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/xOlQ8cu5EFI3-tH4I0iwL002sbbdQeJj3gQWOhkRoXTh1Bm48LiFFaVrbsQYX4WmTOnr4pTuc0pk0uPN2Hn1GURh4XfKwF8U31xH4CXIPJajfOMmLxDEVV-IMNTDEDiH-oxd8QlCebqCtDyw2UIv6kn2n3Y1DFZtpDi14O9I5FlnUgROLFwdKlMyKOJ0pVDe?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/joPNnp28JOL4S2-FqicKRrjb9fAXqQ1fkD3_HhX36GFTywWkKwjybtm8NF902IDMTH0j916OdPmSzLqtAF1nKi1t5FE5KHVAJ4icuYGkFlhfaN6NoZkJubkm_IqhaTTPqgLWzeUBwErTxzsYXiXaGUGLTI1WPxY5n1w23WEniDUELpuiW6gXaWekQgR4xfSP?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/-eB3aaQr8fRgupH0NksNY6llJ9oW36WQotJDPBhw1syJedqezYkJTHtTHpNRvwrgcJiE1W1GBY18o29-IgkKxWskVIsrHf-1oggFCOJfSxeiKWLf8hBifVaSafOzZDEZdL90prTwjtISWHQexSTRC5EWIcDZhtP6GSmDiEvMVL_qI0qI_PEYkMlIYQ3OTl8_?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/1Ap0yhfV9wfwhLNOBOXhFb8aj5xfjE1TykhCtbnOweM-rZgBSeufIozeAbdZ-VeaWA7YaCxM98UtOA7G9goN-EIYxZmjDWoBfzUfBElH9dIWLNgPqq7WWk0bTj_yasUlJdftqZBa3Wog7XAslkDapsPxb5IKYcMTXLiFP5gC4bgQ1fEyBjU5E3C867ylKBj-?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/aAuHCA39mtjLqfFhC3v2zGedqwqnrJ-iELiU2Eb6R6mpKAz5wiHf4FycBMbDjAxSfFPED3kuiaknRMcTkFONzyJVFSgV27yQmuv4SdGFbKcEjWXDKv3ddRFyLiQHaDused4plsrXIiuOETglBZfCaBvsA4VA_u05DNPhhlJG0-Xv_-dB8bJv0OK9g4gpDUnm?purpose=fullsize">

### 📌 Description

Overall system design showing how different modules interact.

### 🔍 Components

* Sensor Layer (Camera + IMU)
* Frontend (Feature extraction, pose estimation)
* Backend (Optimization, loop closure)
* Output (Pose + Map)

### 💡 Importance

Shows how raw sensor data becomes:
👉 Accurate drone position

---

# 📦 Figure 4: EuRoC MAV Dataset

<img src="https://images.openai.com/static-rsc-4/yUkApLjn9yB2Rt2np4aejgoQgDKaXSh9I5LdhXdeCf4sDDCCdWLjrsjS0tKZq0ACA3YTz54R7VKscxzrIHW0NCTve9P8jRtasiAVmwSchsGhGwDrdbBnTji19vxtkK-1rXn2JTTXXbA4w9FTGfAbCkCAfFKq7YJKiWlaSSjbuY3Iw4pq2qnriGZBh63MgKV-?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/vcMsmPPZYyOaqKZRPnbMG2ShzPOEkfY7BEFIwU7Rzfb8gNR2FxEtdgwNYonIxKFrSfGrjnpYsSkdK5-OxD_OX8muiA3PzZhhL3OFi8rAB3M6DTDSTzAu6H8BNqcdQvrbkxHNi9Es5IU9knfYvWDOyj0s4JYM2oFrVYQsHAD81F4UFbRPEA-AWghRXlSuPVCi?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/_cYN9URx_OsBtnos_PBCZDkCUCDuaHxmJKgPFbj8fqBup_5AyA9M4wr0nbcuAmaR3ImM0wfBiizT6iGtKQDwiLofbYQhhQfeG9S0ZCO6uBVbtAYNWQHscGtYHJxVOL70w5OySbunP3kzuzuPQpdhpeGlcyGWfb2vmbDCDAvbsRiwBp--JnUfKnj9B3NKFR1O?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/4LAJpy1b51itskaW3iVKGtGoMqkWOykSv_meC9Ahr83vWAivwpG5fTPpusfvi8vbp-5Y994vlyF35Gw5IRsPdmn1Q7Zdd8PaEMzGM6-qJWrYCu39rUEoPhh99vcfAXrENHFl5R7WVaSUCX2Q99CFG76uXEujHdcxyNe0Z8lbwVYe66sMjH8q2aM_jRxngYl0?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/qoqFwnUlcnf9GApvEzO80Mta-BvbywosC2ArdtmCjS_0bDiOcoC9cO01u7nG5A-1qkxi7hEe397mUWfPup7hk-ZPgnuVAaxG15_R5RG3nJTWBQm0KAqgyFHVL_M95ODKKSuQyGSSpmbF0OHcmJiwH0LuD_MqFSqYomae5lTmhGpB36IPY7xArsHOPp0frKAx?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/iisih30BQE1xWhVagNmzDKCntRbjBK8maIQGwDOrWHPYoECxTttAhFrt8x2lXD5Jb8hI8g0hcmbsRe-_SxhAiaDCjLJlns0zKyYjZMU9Ea1-c9_BcOsQhNCVtCilE29k9rPxtZd7Yyb_jaqpuQ_e66bfvC3-gf4Fc6ElMHuNaCgbl2xIS2N2m6oeX1jv6US1?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/9e6wGSlptHMKTrFA5gMACdwAq28zfaxPvN80SF1ousXAetIDi8bAyr3jFqjLaSHCMgAK_1xRhB7g6ZN6_9cT-OqaQIYOhXCQDi2gTif_B9D4VoMwRvssw4oDzCAb3oNGJfCQkbrDKVsVPehf981y76r2uh4WCFMnzJiPBHsc4moND49x91cpqpx7uu6UquDP?purpose=fullsize">

### 📌 Description

Illustrates the dataset used for testing the algorithm.

### 🔍 Contains

* Stereo images
* IMU readings
* Ground truth trajectory

### 💡 Importance

* Standard benchmark dataset
* Used in ORB-SLAM3 paper
* Enables accuracy validation

---

# 🛬 Figure 5: Autonomous Landing Pipeline

<img src="https://images.openai.com/static-rsc-4/Wk74LZ3Qe7cnRZMY1syas4UZgEsIRKeoNIrXnhu7aXsI7ZnyWn1QSijOnixUGTXMRT2oyi4DJjm1V1gfostF6yMi2s9CkpqAmwvaXxYvla4u6o-jC9ZJ9MSoD_5ROt4Qa45hVBPxLNT5XBuWJXFUPC58wUncE1lHdi377CHzcZgrBwp8CXwUXo4R_KrVdlPr?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/o8lTiqingA5TXNoAqCjzHZg2inVeuRXmJkYgEd9ZxiJfcMsl9VLkurVNDzIaLqlaDarmzYUoYPOzyarmcs70Sf59g81csJt4QFxPrLn7KoPEevoU8-G_-Pc2clnhoYsfFUjCgl215-DMY1hHrZLrHn_8tJXlQPNiYLe6-ougEgo75U8GyD4jZnT53UG1oNLS?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/YAOKaeTPgFs5v_8-VZwL1wSuiRhfo0Fn-ftXYzOe7_f8TzF4fCN0VkHNV99wESd_MT5VVC3EwU5niVSBSqEKRAEmTevdeItXoeWa38qe5soaltUeNzciMdyWhwWmmC0rYroJAHLUjXK-FqYXF7fpHDsANFd8GBbdMpIGPb7VCCp8HWdD9WHCtR745ojaoH56?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/_BR2xGKnNjeYJSeAiI8To8jhjTMgsoX-Wdy2YGw0Irut2NQdOr0xhmUTjc67RsqlGoYtNIYJeLQ-VO6Fka9U-m98xSyuhfsLkr2vEXEVNVJhTBaB0ZlnjK1HkC3yRHb8mV5eOAw7d2eJP4VwtrLOLAt0K__XAfJOon7gKYQYGcKOrnTvXApA6-55VDOLi47c?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/35kEYRHHae2PbIyD4uVrL9dU3R5Yz_rLoEm1buwOnc7TKlxw8SVGBij1jmYxlnFofB3ezkeSw8HXjFLsye8ZeqG8irL4NIOukfP_iRgYD9M7cJPREPwQP5d05XrtiGo7k1yRJ1vsrqkmhKlqGyphu18uXsz7j_RaD-xOFresApEaXC9ijPC07bS1wCzbj1c3?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/uP8ycyML7N6vjQtt8aGXo-CUXRylSXSqEZTnjipUAaBAIC8kexW-p53TyWcnk3jHa6nVnx1BlGzFt7rE61ryLOP0e-NCx6GL3HyJVm3sOmZDYvqIVqo0V-jEzlGKdYeNdNsahS437l_tif7p8phipUV2mKNV5T8PM7Rt4MfyOzVdW32J6PbPpA_Ojf0BWOKM?purpose=fullsize">

### 📌 Description

Shows how localization fits into the full landing system.

### 🔍 Layers

1. Localization (this project)
2. Target Detection
3. Control & Landing

### 💡 Importance

Localization is the **foundation layer**:
👉 Without it, landing is impossible

---

# 📊 Figure 6: Trajectory Visualization

<img src="https://images.openai.com/static-rsc-4/XvA-1hJ08FzbmV1DPXGjGXaWGdc7GznqcVMorU0bABJX1TICoRnWpUzDrkoVkiUg9gMr65lrs1YNcUtGeuvQY6EO2r8Aiyy79uvNcfYdlmAs2VCArv63xtZK9i921cYPLgO0iEmeuvqRJT_lJUOayZTkS9YXtLxFo00jEe-FUoz8whjbJ8Argx1zZg3VhzXN?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/Ji8yhMT_XxlyWz6VBxtCkyG0XOeoevduS_1f0jX3rNil72-l1ZsLvVuD6uXeILOZd-BYVyIH4mC0Z6nwH7X2inYFkJGaPIGI3BXslwZRbNBgkMj4c1Mu06zBFc5qyz4KBCWNbXNgSNVzqf6aRESPxdAGbOkjtjB3nJPkwVaSr9jWpSh-MJOV8CGSlI-wCwxi?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/RxDt9KQ_wO_1X3G2U9oks7kJqFRrxxqt5C7o3hGnFU7ddS4UPRvgTclmrnCjqZAhPfg5xVo8rH3aaiv_EY-diNgk2TiRxjSLb9A2fMLDjvUjN89wq3RzZQKLztm8z3LCk1VhH-d8oTa_JNPD3WJtKIRqtY7lA3W1Wm4O_lLmcCVQLyz-QCskNIRBfy-6YXd6?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/zcmFFUKEo7h_XiVTcraHHu5Wu2T7VD0YVZVtEy46WkBB0tGRJ1XbUzfVsp_Pnr5Ccbarkfi2Np17mnDdvxDxc4yMSa3TaabSUb9owz7Wcy3Ad2hxWk-tQ7J3HbOR6tiGJrCmB7-e0-XpX_YCmbvyJViwanlvDckyEOnhPHdfLjUQQYfv5ttBufKmuUnuCd3h?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/I0AamMflQdGblRSnVR1viCIMGELCn-lGkM_UZIofftuhlRHDp2S3jOMxoo4dM6R80RW8h-45aimwlQL8JTPrhzKjgv-HnCoNfbETDW07Sv2F33KHETvW4CtnToQUf13zVvAwhJ16RdokBWm2db9iIPvgty-z9oqqaGSjML6GeIpf0ecbRzCP7cX7gVKxgxTS?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/5tAYZimtha-7Klm_LG3KDXjzllQjyMiQUzJpqqyfCVnV9fe1wy88VCCfvENTzW8Vx3nEiF76Pox85fzIfwlHjgMzBcFVGvyZWyUdXs3I9Rq0-Ep_cuob10ClMIJDCRmz4U2_OJqnlW0BGzmSmQdThlcQ-Du6AMle8pWSaGlAiYFWpUOqQdq1I1K9BGMJ7i2c?purpose=fullsize">

### 📌 Description

Shows predicted vs actual drone path.

### 🔍 Elements

* Estimated trajectory (blue)
* Ground truth (red)
* Error visualization

### 💡 Importance

This is the **main proof of correctness**

---

# 🔁 Figure 7: Loop Closure Concept

<img src="https://images.openai.com/static-rsc-4/dnvquZ3wRUFBhao9EapXgwAhmVu1ls8nhuopq5ktIQQedb3MsG5R__0vzDG3lQ-LVxi-xypmNpjqAihEfaSK2PGIksczHrniFtTvqkyNpPpmNvUIsyFBGUa1uMvtx5pHVmOu6dzIawuT5TJMsVMQIWhH2_hAbKeqYrRevh7F7tAzBmF9xpJ77zkHa3cDDMJn?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/2voUr3j5JxQFkBZBzNnoZL3m-hfyTASIe4m8r5gtOQPKoXD07glXAlIxnPiNahcPNIYhbm_XPpIjaeLyvLKvG8LI_GkXtahUmyEcLzyY2HKbr_WTCEujTRg2zr9DfceExsJ_X3X-uOengfuiG5k9srWPPPg4Mpfu0AbwYFbIAgMMDn80IdO1WN7QiY-eBDxE?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/Pgz4xsys-uX4xdeb27VPLEp9RpF9PNK1Q0KoPvNacBuMrfXqFWmy7LlvzhRIHWtPKZWe5hy4VVEkvkNtsbPWqwLVEVKJOcyn8MYZzVLvkJAiEmIqmK8gAZWdutvsuZK3uTJl_6QEhIFaKmsV36aMomNzrnLSgG0If8mg8WSIyFrv50AxO_zRt2_PnxXUS5oY?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/7FISXYD7WEPTFaBZsGCgjfSv3BdOaQVtBim-YTFFzQeVCL35ErEiaoqqPIFPt-413Q8ZOIc1wHJ0gAEuwiC8vwHb4RPhf2R792PkLCvumpmoMXzM58GHrmfMPx98rSGwpec61l4Wz8Pu-AKUiAavlLoCgDGun2L8asjcaBbORLB1E8HgQ1s9R_CWs1ZgUMlp?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/0q8rF3YYb_pLN9Sc7LKc4RWKEMnoXpFxQoXx2AR7SY0XtkGpUxDrSRcOTif8zSHGYCdcyy4CXIiGQj4kQY292B9gRBQXKkB0ZnLjJSyNC5zHmYzt_k2iR_hiKOgVlp5XvfO56fRUQaL1f2D3DgaZGz6DH3g-Sl_Pmy9RMqz1q6ysqCpnSdwPNugLEp8U1YOz?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/whlIlfqbL7dqL27GdBPt28WbVfnoKgo5oQcHweDqvH1a0nYLedgJHrJQYUX68P09Clkj3UE_1Fpif9W1iUamRmfg1OfG6nD3aRPCgt1rTwXYCzyxqZB2ovDvOqHadAF-oZUE-BHf09UVQJJqCrgWJjJJm0_-qFFEOsQlXbOuHS5jj8vZ5Aejks3Fb8KZaKiR?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/tBRKwaAhoGV4ljS9xChpOSbLComHQn8keCZP7e3F1LP0IQ1gr0r1zcA233MlnoFX1MadUobXDfIJG7K0iyxuP-CEFzejIlC1Sa6ObHqbkqP0M_tosITZJeXiJpnPtUbXU11Lv8DnIiCJv1OhDicPppSdz0VCgoabrbF5M2CRfOggMBuaWiFpnkCv45NJUUBm?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/IWmR4GzLM_PDydHlWDcv6Qfq0R4M8DuOJztoy7dMnluS7VMqmod0ROFrjY9z7hcV-6be1OI53eXMKG9Y_M1kP59hxMNkspIuAq79PlIJZGpFXz5go6uDZfF2Gm5HVRMA0W5AmSO2ZiOFlkSopDiW9K5D3sPuGc06H7n_wZrtghJE1qgz6ncOkwGdU0MgMYgW?purpose=fullsize">

### 📌 Description

Explains how the system corrects accumulated error.

### 🔍 Process

* Detect previously visited location
* Adjust entire trajectory
* Reduce drift

### 💡 Importance

Enables:
👉 Return-to-origin capability

---

# 🧩 Figure 8: Project Workflow

<img src="https://images.openai.com/static-rsc-4/eFBPRT-gKi2FPUCV0zmwazQWUWN5-BE2Nb7YbO24uq5wzErv5Qo5G2-gt_6YT4qI4_nF6ebuaPLVLbdGcYQPt9_BBehvPENCJY0vxzNPVrAkp_NjxzAcQ6QBXqy0lBiIgCUPKbAyd3hQU_vytqW8hFXwQYYHrProGsezMK1dooc8Lr-WlFQup6pDNXg1CqnW?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/QX0JzKlgBoNNkcX9dapxnooyfhE40B1VWdKkaH5fN-3e8M5K0QUunTE44qHVcz559I_csFbQOLTS9QKNNi8yozGmjC1M_ODMZhjhlL7y6VnBjWmRZ3HJmDzccTEu3_h7Sv2K3CjhH-CbhLa5jU5KTAwkgTkTZ5hrwJcYaFERWe0iVtbxRG1NvatgQSYqI9Ds?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/czArz7XHJU2wW8VfGkReEPKJEduLvL_adSLjcQWA7nHmHSAgSR-I0Y_f_xEt4DsS3GALdW5Q3MohRYgdRPHjapnpz55AxDT-A0D4-0O76CCGBYRxdRHVGs1EKDFPoNhgrKLTZe7MDW3-0lp0QHWTMylJyTLqcrlq3VL4kdKjvUlg4UL0zagnkeJ4UtwV7JJA?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/tGehixr8j_KJf1To6OFSF3LYKa_CdQNa8zzp6L7YLiypRcm3OZ9aoxN2Qcybpi8bKcBXs-zEct4EX8qCYIF1zgh-P7KGiQooi6Yj6kM_qdiykG40dOPrFD0FmTlQIdvn5LosjWc3kIhMecmLOCqRNLrnNEwXbgl5x7z6ZmqUjn6ovBnxlDA8Aa7S7U0DYlkh?purpose=fullsize">

<img src="https://images.openai.com/static-rsc-4/hOuyi7uxcMp56qQZkCT8IxNv388EzV6cKrF5kq1B9o2qxfhNlyg5cotpFj_w9YkQI3kE7Hyob4aXwR_Ls443O7fNDGiZgiMxg_3O_FJER55vD1Jd9n-UmatNfpe88WifyxVZsbjGq0szu3W60NKRQuUiAw6I6r9Ol6ty65l1R5HnzGuJAAXf9X-m2HGwpnAd?purpose=fullsize">

### 📌 Description

Shows complete workflow of the project.

### 🔍 Steps

1. Idea
2. Dataset
3. Algorithm
4. Simulation
5. Evaluation
6. Deployment

### 💡 Importance

Helps understand:
👉 How to build system from scratch

---

# 📌 Summary

These figures collectively explain:

* How localization works
* How the system is built
* How results are validated
* How it connects to real-world applications

---

## 🎯 Usage

These images can be used in:

* Research report
* PPT presentation
* GitHub README
* Viva explanation

---

## 🧠 Final Note

Understanding these visuals is enough to:
👉 Fully explain the project to anyone (even non-technical audience)
