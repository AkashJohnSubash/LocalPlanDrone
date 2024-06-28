import casadi as ca

dS = 0.5

s = 0
gam_y = 0
gam_x = 0
gam_phi = 0

# generate straight section
K = 0
for i in range(29):
    print('   {:e}'.format(s), '   {:e}'.format(gam_x), '   {:e}'.format(gam_y), '   {:e}'.format(gam_phi), '   {:e}'.format(K))
    gam_x = round(ca.cos(gam_phi) * dS + gam_x, 7)
    gam_y = round(ca.sin(gam_phi) * dS + gam_y, 7)
    gam_phi = round(K * dS + gam_phi, 7)
    s = s + dS

# generate 3/4 circular curve interpolation points
K = - 0.25
for i in range(33):
     print('   {:e}'.format(s), '   {:e}'.format(gam_x), '   {:e}'.format(gam_y), '   {:e}'.format(gam_phi), '   {:e}'.format(K))
     gam_x = round(ca.cos(gam_phi) * dS + gam_x, 7)
     gam_y = round(ca.sin(gam_phi) * dS + gam_y, 7)
     gam_phi = round(K * dS + gam_phi, 7)
     s = s + dS

# generate straight section2
K = 0
for i in range(30):
    print('   {:e}'.format(s), '   {:e}'.format(gam_x), '   {:e}'.format(gam_y), '   {:e}'.format(gam_phi), '   {:e}'.format(K))
    gam_x = round(ca.cos(gam_phi) * dS + gam_x, 7)
    gam_y = round(ca.sin(gam_phi) * dS + gam_y, 7)
    gam_phi = round(K * dS + gam_phi, 7)
    s = s + dS

# generate 3/4 circular curve interpolation points
K = 0.25
for i in range(35):
     print('   {:e}'.format(s), '   {:e}'.format(gam_x), '   {:e}'.format(gam_y), '   {:e}'.format(gam_phi), '   {:e}'.format(K))
     gam_x = round(ca.cos(gam_phi) * dS + gam_x, 7)
     gam_y = round(ca.sin(gam_phi) * dS + gam_y, 7)
     gam_phi = round(K * dS + gam_phi, 7)
     s = s + dS

# # generate 3/4 circular curve2 interpolation points
# K = - 0.15
# for i in range(65):
#      print('   {:e}'.format(s), '   {:e}'.format(gam_x), '   {:e}'.format(gam_y), '   {:e}'.format(gam_phi), '   {:e}'.format(K))
#      gam_x = round(ca.cos(gam_phi) * dS + gam_x, 7)
#      gam_y = round(ca.sin(gam_phi) * dS + gam_y, 7)
#      gam_phi = round(K * dS + gam_phi, 7)
#      s = s + dS