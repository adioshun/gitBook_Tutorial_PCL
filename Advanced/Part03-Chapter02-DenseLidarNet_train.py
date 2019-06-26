from __future__ import print_function

import os
import argparse
from pdb import set_trace as brk
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import time
#from logger import Logger
import shutil
import sys

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
from torch.autograd import Variable

import torchvision
import torchvision.transforms as transforms
from dataloader import DenseLidarGen

from chamfer_loss import *
from model import *

def save_checkpoint(state, is_best, filename='checkpoint.pth.tar'):
	torch.save(state, filename)
	print("\n Model Saved :",filename,"\n" )
	if is_best:
		shutil.copyfile(filename, 'model_best.pth.tar')

class Main(object):

	def __init__(self, args):

		self.batch_size = 20
		self.args = args
		self.max_pts_in_voxel = 20
		#normalize  = transforms.Normalize((0.485,0.456,0.406), (0.229,0.224,0.225)
		self.transform = transforms.Compose([transforms.ToTensor()])
		self.train_dataset = DenseLidarGen(args.train_lidar_pts_path, args.train_tf_lidar_pts_path, args.train_bbox_info_path, self.transform)
		self.val_dataset = DenseLidarGen(args.val_lidar_pts_path, args.val_tf_lidar_pts_path, args.val_bbox_info_path, self.transform)
                
		self.train_dataloader = torch.utils.data.DataLoader(self.train_dataset, batch_size=self.batch_size, shuffle=True, num_workers=1, collate_fn=self.train_dataset.collate_fn)
		self.val_dataloader = torch.utils.data.DataLoader(self.val_dataset, batch_size=self.batch_size, shuffle=False, num_workers=1, collate_fn=self.val_dataset.collate_fn)
		
		# self.load_model()
		self.num_voxels_z = 20
		self.num_voxels_x = 10
		self.vfe_embedding_size = 128
		self.use_cuda = torch.cuda.is_available()
		self.load_model()
		self.criterion = ChamferLoss()
		self.optimizer = optim.SGD(self.net.parameters(), lr=args.lr, momentum=0.9, weight_decay=1e-4)
		self.run_time = time.ctime().replace(' ', '_')[:-8]
		directory = 'progress/' + self.run_time
		if not os.path.exists(directory):
			os.makedirs(directory)
		#self.logger = Logger('directory')
		self.num_gpus = torch.cuda.device_count()
		self.gpus = np.arange(self.num_gpus).tolist()
		self.gather_device = self.gpus[0]

	def plot_stats(self, epoch, data_1, data_2, label_1, label_2, plt):
		plt.plot(range(epoch), data_1, 'r--', label=label_1)
		if data_2 is not None:
			plt.plot(range(epoch), data_2, 'g--', label=label_2)
		plt.legend()

	def load_model(self):
		print("\n Model Loaded \n")

		self.net  = DenseLidarNet()
		self.net.load_state_dict(torch.load('./scripts/net.pth'))
		# assert torch.cuda.is_available(), 'Error: CUDA not found!'
		
		# self.net = torch.nn.DataParallel(self.net, device_ids=range(torch.cuda.device_count()))
		if self.use_cuda:
			self.net.cuda()
		
	def adjust_learning_rate(self, optimizer, epoch, base_lr):
		"""Sets the learning rate to the initial LR decayed by 10 every 30 epochs"""
		lr = base_lr * (0.1 ** (epoch // 400))
		for param_group in optimizer.param_groups:
			param_group['lr'] = lr

	def prepare_gpu_input(self, all_voxel_data, all_voxel_mask, all_voxel_indices):
		
		batch_size = self.batch_size
		num_gpus = self.num_gpus
		batch_per_gpu = batch_size/num_gpus
		batch_voxel_features = []
		batch_voxel_mask = []
		batch_voxel_indices = []
		batch_scatter_ops = []
		templist_1 = []; templist_2 = []; templist_3 = []; templist_4 = []

		inc = 0
		
		for it,(blob1, blob2, blob3) in enumerate(zip(all_voxel_data, all_voxel_mask, all_voxel_indices)):
			#which_gpu = self.gpus[it//(batch_size//num_gpus)] #For python3
			which_gpu = self.gpus[it/(batch_size/num_gpus)] 
			templist_1.append(Variable(blob1.cuda(which_gpu,async=True)))
			templist_2.append(Variable(blob2.cuda(which_gpu,async=True)))
			blob3 += self.num_voxels_x*self.num_voxels_z*inc
			templist_3.append(Variable(blob3.cuda(which_gpu,async=True)))
			inc += 1
			
			inc %= (batch_size/num_gpus)
			
			if inc == 0:
				
				batch_voxel_features.append(torch.cat(templist_1,0))
				batch_voxel_mask.append(torch.cat(templist_2,0))
				batch_voxel_indices.append(torch.cat(templist_3,0))
				batch_scatter_ops.append(Variable(torch.zeros((batch_size/num_gpus) * self.num_voxels_x * self.num_voxels_z, self.vfe_embedding_size).cuda(which_gpu,async=True)))
				templist_1 = []; templist_2 = []; templist_3 = []; templist_4 = []
		return zip(batch_voxel_features, batch_voxel_mask, batch_voxel_indices, batch_scatter_ops)
	
	def customdataparallel(self,zipped_input):
		# Distributes the model in multiple gpus
		replicas = torch.nn.parallel.replicate(self.net,self.gpus)
		# Distribute the input equally to all gpus
		outputs = torch.nn.parallel.parallel_apply(replicas,zipped_input)

		# Gather the output in one device
		return torch.nn.parallel.gather(outputs,self.gather_device)
	
	def train(self, epoch):
		print("\n TRAIN MODE \n")
		train_loss = []
		self.net.train()
		for batch_idx,(voxel_features,voxel_mask,voxel_indices, chamfer_gt) in enumerate(self.train_dataloader):
			zipped_input = self.prepare_gpu_input(voxel_features, voxel_mask, voxel_indices)
			if len(zipped_input) == 2:
				hallucinations = self.customdataparallel(zipped_input)
				chamfer_gt = torch.FloatTensor(chamfer_gt)
				loss = self.criterion(hallucinations, Variable(chamfer_gt).cuda(self.gather_device))
				train_loss += [loss.data[0]/chamfer_gt.size(0)] #For pytorch <= 0.5
				#train_loss += [loss.data/chamfer_gt.size(0)]
				if batch_idx % self.args.print_freq == 0:
					progress_stats = '(train) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {net_loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.train_dataloader), net_loss=loss.data[0])
					print(progress_stats)
				self.optimizer.zero_grad()
				loss.backward()
				self.optimizer.step()
			else:
				return train_loss
		return train_loss

	def validate(self):
		print("\n VALIDATE MODE \n")
		val_loss = []
		self.net.eval()
		for batch_idx,(voxel_features,voxel_mask,voxel_indices, chamfer_gt) in enumerate(self.val_dataloader):
			zipped_input = self.prepare_gpu_input(voxel_features, voxel_mask, voxel_indices)
			if len(zipped_input) == 2:
				xyz_output = self.customdataparallel(zipped_input)

				### save tensor torch to numpy 
				a = xyz_output.detach().cpu().clone().numpy() 
				a.tofile("xyz_out.bin")
				print("xyz_out File saved\n")
				
				#hallucinations = self.customdataparallel(zipped_input) #typing error?
				chamfer_gt = torch.FloatTensor(chamfer_gt)
				loss = self.criterion(xyz_output, Variable(chamfer_gt).cuda(self.gather_device))
				val_loss += [loss.data[0]/chamfer_gt.size(0)]
				if batch_idx % self.args.print_freq == 0:
					#progress_stats = '(val) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.val_dataloader), loss=loss.data[0])
					epoch = 0
					progress_stats = '(val) Time: {0} Epoch: [{1}][{2}/{3}]\t' 'Loss {loss:.4f}\t'.format(time.ctime()[:-8], epoch, batch_idx, len(self.val_dataloader), loss=loss.data[0])
					print(progress_stats)
			else:
				return val_loss
		return val_loss
	

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Dense LiDarNet Training')
	parser.add_argument('--lr', default=1e-9, type=float, help='learning rate')
	#parser.add_argument('--resume', '-r', default=False, type=bool, help='resume from checkpoint')
	parser.add_argument('--epochs', default=1, type=int, metavar='N', help='number of total epochs to run') # default=10000
	parser.add_argument('--start-epoch', default=0, type=int, metavar='N', help='manual epoch number (useful on restarts)')
	parser.add_argument('--print-freq', '-p', default=10, type=int, metavar='N', help='print frequency (default: 10)')
	parser.add_argument('--resume', default='', type=str, metavar='PATH', help='path to latest checkpoint (default: none)')
	parser.add_argument('-e', '--evaluate', dest='evaluate', action='store_true', help='evaluate model on validation set')
	parser.add_argument('-tp1', dest='train_lidar_pts_path', default='../data/lidar_pts', type=str, help='Train Lidar Pts Path')
	parser.add_argument('-tp2', dest='train_tf_lidar_pts_path', default='../data/tf_lidar_pts', type=str, help='Train TF Lidar Pts Path')	
	parser.add_argument('-tp3', dest='train_bbox_info_path', default='../data/bbox_info', type=str, help='Train BBox Info Path')
	parser.add_argument('-vp1', dest='val_lidar_pts_path', default='../data/lidar_pts', type=str, help='Val Lidar Pts Path')
	parser.add_argument('-vp2', dest='val_tf_lidar_pts_path', default='../data/tf_lidar_pts', type=str, help='Val TF Lidar Pts Path')																												
	parser.add_argument('-vp3', dest='val_bbox_info_path', default='../data/bbox_info', type=str, help='Val Bbox Info Path')        
	args = parser.parse_args()
	
	print ("****************************************************************************************")
	print ("Using Learning Rate     ==============> {}".format(args.lr))
	print ("Loading from checkpoint ==============> {}".format(args.resume))
	print ("GPU processing available : ", torch.cuda.is_available())
	print ("Number of GPU units available :", torch.cuda.device_count())
	print ("****************************************************************************************")

	net = Main(args)

	train_loss = []
	val_loss = []
	best_loss = np.inf
	if args.resume:
		print("RESUME MODE")
		if os.path.isfile(args.resume):
			print("=> loading checkpoint '{}'".format(args.resume))
			checkpoint = torch.load(args.resume)
			args.start_epoch = checkpoint['epoch']
			best_loss = checkpoint['best_loss']
			net.net.load_state_dict(checkpoint['state_dict'])
			net.optimizer.load_state_dict(checkpoint['optimizer'])
			train_loss += checkpoint['train_loss']
			val_loss += checkpoint['val_loss']
			print("=> loaded checkpoint '{}' (epoch {})"
				.format(args.resume, checkpoint['epoch']))
		else:
			print("=> no checkpoint found at '{}'".format(args.resume))

	if args.evaluate:		
		val_stats = net.validate()
		print("VALIDATE : avg loss = ", np.mean(val_stats))
		sys.exit(0)

	for epoch in range(args.start_epoch, args.epochs + args.start_epoch):
		net.adjust_learning_rate(net.optimizer, epoch, args.lr)
		
		# uncomment following two series of code block to verify if backpropgation is happening properly
		old_params = []
		for i in range(len(list(net.net.parameters()))):
			old_params.append(list(net.net.parameters())[i])
		old = time.time()    

		train_stats = net.train(epoch)
		train_loss += [np.mean(train_stats)]
		print("time.time() - old:", time.time() - old)
		for i in range(len(list(net.net.parameters()))):
			print("weight update for parameter : ", i, not torch.equal(old_params[i].data, list(net.net.parameters())[i].data))

		val_stats = 0
		val_stats = net.validate()
		val_loss += [val_stats]
		print("VALIDATE avg loss = ", np.mean(val_stats))
		
		# remember best val loss and save checkpoint
		is_best = val_stats < best_loss
		best_loss = max(val_stats, best_loss)
		save_checkpoint({
			'train_loss':train_loss,
			'val_loss':val_loss,
			'epoch': epoch + 1,
			'state_dict': net.net.state_dict(),
			'best_loss': best_loss,
			'optimizer' : net.optimizer.state_dict(),
		}, is_best)
		
		plt.figure(figsize=(12,12))
		net.plot_stats(epoch+1, train_loss, None, 'train_loss', 'val_loss', plt)
		plt.savefig('progress/' + net.run_time + '/stats.jpg')
		plt.clf()
