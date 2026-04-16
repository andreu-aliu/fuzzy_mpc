set pagination off
set print pretty on
tui enable

break MPC::compute_mpc
break MPC::createModelMatrices
break MPC::solve
break MPC::predict_states