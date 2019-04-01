

# helper classes

class PID_controller(object):
	def __init__(self, pGain, iGain, dGain):
		self.pGain = pGain
		self.iGain = iGain
		self.dGain = dGain
		self.prevError = 0
		self.iError = 0
	def update(self, error):
		dError = error - self.prevError
		self.iError = self.iError + error
		control = self.pGain*error  \
				+ self.dGain*dError \
				+ self.iGain*self.iError
		self.prevError = error
		return control			
		

# helper functions

def blocksAreNew(oldCount, oldBlocks, newCount, newBlocks, verbose=False):
	if oldCount == newCount:
		for i in range(oldCount):
			if not ((oldBlocks[i].m_signature == newBlocks[i].m_signature) \
			    and (oldBlocks[i].m_x == newBlocks[i].m_x) \
			    and (oldBlocks[i].m_y == newBlocks[i].m_y) \
			    and (oldBlocks[i].m_width == newBlocks[i].m_width) \
			    and (oldBlocks[i].m_height == newBlocks[i].m_height)):
				# return true because the current block is different
				# from the corresponding block in oldBlocks	
				if verbose:	
					print "block is different"					 
				return True	 
	else:
		# There is a different number of blocks so ther must be new blocks
		if verbose:
			print "different number of blocks"
		return True		
		
	if newCount == 0:
		# no new blocks because there are no blocks
		if verbose:
			print "no new blocks"		
		return False
	# The number of blocks is identical and all the blocks are identical
	if verbose:
		print "all blocks are identical"
	return False
	
def copyBlockArray(srcCount, srcBlocks, destBlocks):		
	for i in range(srcCount):
		destBlocks[i] = srcBlocks[i]

