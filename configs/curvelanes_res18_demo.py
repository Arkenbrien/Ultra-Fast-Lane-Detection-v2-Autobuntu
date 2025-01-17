dataset='CurveLanes'
data_root= '' # Need to be modified before running
epoch=100
batch_size=1
optimizer='Adam'
learning_rate = 4e-4
weight_decay = 1e-4
momentum=0.9
scheduler='multi'
# steps=[25,38]
gamma=0.01
warmup='linear'
warmup_iters=100
use_aux=True
backbone='18'
sim_loss_w=1.0
shp_loss_w=0.0
note=''
log_path=''
finetune=None
resume=None
test_model=''
test_work_dir=''
tta = False
num_lanes=10
var_loss_power=2.0
auto_backup=True
num_row=72
num_col=41
train_width=1600
train_height=800
num_cell_row=200
num_cell_col=100
mean_loss_w=0.05
crop_ratio = 1
