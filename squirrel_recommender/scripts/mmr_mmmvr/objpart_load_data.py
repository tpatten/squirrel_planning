## ##################################################
import numpy as np
import scipy.io

## import scipy.linalg as sp_linalg
import mvm_prepare
import mmr_kernel_explicit
import mmr_kernel_subspace
from mmr_initial_params import cls_initial_params
import mmr_kernel_eval

## ###################################################
## ##################################################
class cls_label_files:

  def __init__(self):
    """
    Type indeces :  Features 0, Object 1, Part 2, Scene 3, Template 4,
                    Object category 5, images 6
    """

    self.sbasedir='/home/szedmak/data/pacman/object_part_recognition/'

    ## dictionary:  (type index 1,type index 2, index) :  \
    ##              ( filename, variable name, version)
    self.dirvar={ 
      (1,1,1) : ('similarity_objects_objects','objects_objects_sp'), 
      (1,1,2) : ('sim_parts_parts_obj_obj_v2','object_to_object_sim'), 
      (1,2,1) : ('objects_parts','objects_parts_sp'), 
      (1,4,1) : ('objects_templates','objects_templates_sp'), 
      (1,4,2) : ('score_scene_template_chordiogram','score_object_template'), 
      (1,4,3) : ('indexTemplates_objects_parts','indexObject_templates'), 
      (2,2,1) : ('sim_parts_parts_obj_obj_v2','part_to_part_sim'), 
      (2,4,1) : ('parts_templates','parts_templates_sp'), 
      (2,4,2) : ('indexTemplates_objects_parts','indexPart_templates'), 
      (3,0,1) : ('labels_scene_7objects','annotationsScene'), 
      (3,1,1) : ('labelsScene_sp','labelsSceneObjects_sp'), 
      (3,1,2) : ('similarity_scene_templates_parts_objects', 
               'matrix_scene_objects_sp'), 
      (3,1,3) : ('scene_labels_v1_2', 
               'labelsSceneObjects'), 
      (3,1,4) : ('labels_scenes_4objects', 
                'labelsSceneObjects'), 
      (3,1,5) : ('labels_scenes_4objects', 
                'labelsSceneObjects_n'), 
      (3,1,6) : ('labels_scene_7objects','labelsSceneObjects'), 
      (3,1,7) : ('labels_scene_7objects','labelsSceneObjects_n'), 
      (3,2,1) : ('labelsScene_sp','labelsSceneParts_sp'), 
      (3,2,2) : ('similarity_scene_templates_parts_objects', 
               'matrix_scene_parts_sp'), 
      (3,2,3) : ('labels_scenes_4objects', 
                'labelsSceneParts'), 
      (3,2,4) : ('labels_scenes_4objects', 
                'labelsSceneParts_n'), 
      (3,2,5) : ('labels_scene_7objects','labelsSceneParts'), 
      (3,2,6) : ('labels_scene_7objects','labelsSceneParts_n'), 
      (3,3,1) : ('similarityScenes','similarityScenes_position'), 
      (3,3,2) : ('similarityMatrix_v2','similarityScenes1'), 
      (3,3,3) : ('similarityMatrix_v2','similarityScenes2'), 
      (3,3,4) : ('similarityScenes_histograms_v1', 
                 'similarityScenes_histogram'), 
      (3,3,5) : ('similarityScenes_4objects', 
                 'similarityScenes_histogram'), 
      (3,3,6) : ('similarityScenes_4objects', 
                 'similarityScenes_histogram2'), 
      (3,3,7) : ('similarityScenes_4objects', 
                 'similarityScenes_histogram3'), 
      (3,3,8) : ('similarityScenes_4objects', 
                 'similarityScenes_histogram4'), 
      (3,3,9) : ('similarityScenes_7objects', 
                 'similarityScenes_histogram'), 
      (3,3,10) : ('similarityScenes_7objects', 
                 'similarityScenes_histogram2'), 
      (3,3,11) : ('similarityScenes_7objects', 
                 'similarityScenes_histogram3'), 
      (3,3,12) : ('similarityScenes_7objects', 
                 'similarityScenes_histogram4'), 
      (3,4,1) : ('labelsScene_sp','labelsSceneTemplates_sp'), 
      (3,4,2) : ('similarity_scene_templates_parts_objects', 
                'matrix_scene_templates_sp'), 
      (3,4,3) : ('data_v2', 
                'matrix_scene_template_v1'), 
      (3,4,4) : ('scene_template_similarity_v2', 
                'matrix_scene_template_v2'), 
      (3,4,5) : ('similarity_scene_template_v4', 
                'matrix_scene_template_v4'), 
      (3,4,6) : ('matrix_template_score', 
                'matrix_scene_template'), 
      (3,4,7) : ('matrix_template_score', 
                'matrix_scene_template_max'), 
      (3,4,8) : ('matrix_template_score', 
                'nrActivations_templates'), 
      (3,4,9) : ('matrix_template_score', 
                'nrActivations_templates_max'), 
      (3,4,10) : ('matrix_template_score_2', 
                'nrActivations_templates_max'), 
      (3,4,11) : ('score_scene_template_chordiogram', 
                'matrix_scene_template'), 
      (3,4,12) : ('score_scene_template_chordiogram', 
                'matrix_scene_template_max'), 
      (3,4,13) : ('score_scene_template_chordiogram', 
                'matrix_scene_template_maxT'), 
      (3,4,14) : ('score_scene_template_chordiogram', 
                'nrActivations_templates'), 
      (3,4,15) : ('score_scene_template_chordiogram', 
                'nrActivations_templates_max'), 
      (3,4,16) : ('labels_scenes_4objects', 
                'labelsSceneTemplates'), 
      (3,4,17) : ('labels_scenes_4objects', 
                'labelsSceneTemplates_n'), 
      (3,4,18) : ('labels_scene_7objects','labelsSceneTemplates'), 
      (3,4,19) : ('labels_scene_7objects','labelsSceneTemplates_n'), 
      (3,4,20) : ('score_scene_template_7objects_scenes_3',
                  'matrix_scene_template'), 
      (3,4,21) : ('score_scene_template_7objects_scenes_3',
                  'matrix_scene_template_max'), 
      (3,4,22) : ('score_scene_template_7objects_scenes_3',
                  'matrix_scene_template_ori'), 
      (3,4,23) : ('score_scene_template_7objects_scenes_3',
                  'matrix_scene_template_ori_max'), 
      (3,4,24) : ('score_scene_template_7objects_scenes_3',
                  'nrActivations_templates'), 
      (3,4,25) : ('score_scene_template_7objects_scenes_3',
                  'nrActivations_templates_max'), 
      (3,4,26) : ('score_scene_template_7objects_scenes_3',
                  'nrActivations_templates_ori_max'),
      (3,4,27) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'matrix_scene_template'), 
      (3,4,28) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'matrix_scene_template_max'), 
      (3,4,29) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'matrix_scene_template_ori'), 
      (3,4,30) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'matrix_scene_template_ori_max'), 
      (3,4,31) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'nrActivations_templates'), 
      (3,4,32) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'nrActivations_templates_max'), 
      (3,4,33) : ('score_scene_template_7objects_scenes_3_featureSet2',
                  'nrActivations_templates_ori_max'),      
      (3,5,1) : ('labels_scenes_4objects', 
                'labelsSceneCategory'), 
      (3,5,2) : ('labels_scenes_4objects', 
                'labelsSceneCategory_n'), 
      (3,5,3) : ('labels_scene_7objects','labelsSceneCategory'), 
      (3,5,4) : ('labels_scene_7objects','labelsSceneCategory_n'), 
      (3,6,1) : ('labels_scene_7objects','labelsImages'), 
      (4,0,1) : ('data_v2','template_histogram'), 
      (4,0,2) : ('indexTemplates_objects_parts_7objects',
                 'indexTemplates_object'), 
      (4,0,3) : ('indexTemplates_objects_parts_7objects',
                 'indexTemplates_parts'), 
      (4,1,1) : ('indexTemplates_objects_parts','indexTemplates_object'), 
      (4,1,2) : ('indexTemplates_objects_parts_7objects',
                 'index_templates_object'), 
      (4,2,1) : ('indexTemplates_objects_parts','indexTemplates_parts'), 
      (4,2,2) : ('indexTemplates_objects_parts_7objects',
                 'index_templates_parts'), 
      (4,4,1) : ('matrix_similarityTemplates','matrix_simTemplates_sp'), 
      (4,4,2) : ('template_to_template_sim', 
                 'template_to_template_similarity'), 
      (4,4,3) : ('template_to_template_sim_v3', 
                 'template_to_template_sim_v3'), 
      (4,4,4) : ('template_template_v5', 
                 'template_template_v5'), 
      (4,4,5) : ('similarityTemplates_differentFeatures', 
                 'matrix_similarity_templates'), 
      (4,4,6) : ('similarityTemplates_differentFeatures', 
                 'matrix_similarity_templates2'), 
      (4,4,7) : ('similarityTemplates_differentFeatures', 
                 'matrix_similarity_templates3'), 
      (4,4,8) : ('similarityTemplates_differentFeatures', 
                 'matrix_similarity_templates4'),
      (4,4,9) : ('similarityTemplates_7objects', 
                 'matrix_displacement'), 
      (4,4,10) : ('similarityTemplates_7objects', 
                 'matrix_orientation'), 
      (4,4,11) : ('similarityTemplates_7objects', 
                 'matrix_pairwise'), 
      (4,4,12) : ('similarityTemplates_7objects', 
                 'matrix_pairwise_orientation'), 
      (4,6,1) : ('indexTemplates_objects_parts_7objects',
                 'index_templates_images'), 
      (5,0,1) : ('indexTemplates_objects_parts_7objects',
                 'category_objects'),
      (6,0,1) : ('indexTemplates_objects_parts_7objects',
                 'imageNames'),
      }

    self.ext='.mat'
  ## --------------------------------------------------
  def load(self,tfile):

    xdata=[[],[],[]]    ## row index, column index , value

    print(tfile,self.dirvar[tfile])

    ddata=scipy.io.loadmat(self.sbasedir+self.dirvar[tfile][0]+self.ext)
    X=ddata[self.dirvar[tfile][1]]
    X=X.toarray()
    (m,n)=X.shape
    if m>n:
      X=X.T
    (m,n)=X.shape
    print('Source shape:',X.shape)

    for i in range(m):
      for j in range(n):
        v=X[i,j]
        xdata[0].append(i)
        xdata[1].append(j)
        xdata[2].append(v)
        
    xdata=mvm_prepare.sort_table(xdata,ifloat=1)
          
    return(xdata,m,n)
  
  ## --------------------------------------------------------------
  def load_mmr(self,cData,tfile_in,tfile_out):

    ddata_out=scipy.io.loadmat(self.sbasedir+self.dirvar[tfile_out][0] \
                                 +self.ext)
    X_out=ddata_out[self.dirvar[tfile_out][1]]
    try:
      X_out=X_out.toarray()
    except:
      X_out=X_out
      
    (my,ny)=X_out.shape

    nview=len(tfile_in)
    dX_in={}
    dX_in_internal={}
    for iview in range(nview):
      if tfile_in[iview][0]!=(0,0,0):
        ddata_in=scipy.io.loadmat(self.sbasedir+ \
                                self.dirvar[tfile_in[iview][0]][0] \
                                +self.ext)
        X_in=ddata_in[self.dirvar[tfile_in[iview][0]][1]]
        try:
          X_in=X_in.toarray()
        except:
          X_in=X_in
        dX_in[iview]=X_in
      else:
        n0=10
        dX_in[iview]=np.ones((my,n0))

    ## internal kernel:  X_in X_internal^{1/2} X_internal^{1/2}.T X_in.T  
    for iview in range(nview):
      if len(tfile_in[iview])>1:
        ddata_in=scipy.io.loadmat(self.sbasedir+ \
                                self.dirvar[tfile_in[iview][1]][0] \
                                +self.ext)
        X_in=ddata_in[self.dirvar[tfile_in[iview][1]][1]]
        try:
          X_in=X_in.toarray()
        except:
          X_in=X_in
        dX_in_internal[iview]=X_in 

    ## (mout,nout)=X_out.shape
    if tfile_out in ((1,4,1),(2,4,1),(3,4)):
      X_out=X_out.T

    ## ---------------------------
    xnobj=np.sum(X_out,axis=1)
    bobj=1*(xnobj>=1)*(xnobj<=7)
    iobj=np.where(bobj==1)[0]
    ## iobj=np.where(xnobj==3)[0]
    

    ## subspace output kernel
    cData.YKernel=mmr_kernel_explicit.cls_feature(ifeature=0)
    ## cData.YKernel=mmr_kernel_subspace.cls_feature(ifeature=1)
    X_out=X_out[iobj]
    cData.YKernel.load_data(X_out,ifeature=0)
    ## cData.YKernel.Kraw=mmr_kernel_eval.tanimoto(X_out,X_out)
    (my,ny)=X_out.shape
    if ny<=12:
      iy=np.arange(2**ny)
      cData.YKernel.Y0=np.array(np.unravel_index(iy,[2]*ny)).T
      cData.YKernel.Y0=cData.YKernel.Y0.astype(float)
      ## cData.YKernel.Krawcross=mmr_kernel_eval.tanimoto(X_out, \
      ##                                                  cData.YKernel.Y0)
    cData.YKernel.ifeature=0
    cData.YKernel.title='output'
    ## setting output parameters
    cparams=cls_initial_params()
    cData.YKernel.kernel_params.set(cparams.get_yparams('kernel',0))
    ## cData.YKernel.prekernel_params.set(cparams.get_yinparams('kernel',0))
    cData.YKernel.crossval.set(cparams.get_yparams('cross',0))
    cData.YKernel.norm.set(cparams.get_yparams('norm',0))

    for iview in range(nview):
      cData.XKernel[iview]=mmr_kernel_explicit.cls_feature(ifeature=0)
      X_in=dX_in[iview]
      if iview==0:
        ## X_in=reweight(X_in)
        pass
      if iview in dX_in_internal:
        X_in=np.dot(X_in,dX_in_internal[iview])
      cData.XKernel[iview].load_data(X_in[iobj],ifeature=0)
      cData.XKernel[iview].title='input_'+str(iview)

    ## setting input parameters
      cData.XKernel[iview].kernel_params.set(cparams. \
                                             get_xparams('kernel',iview))
      cData.XKernel[iview].crossval.set(cparams.get_xparams('cross',iview))
      cData.XKernel[iview].norm.set(cparams.get_xparams('norm',iview))

    cData.ninputview=nview  ## set active views
    cData.mdata=cData.YKernel.dataraw.shape[0]

    return
## ###################################################
def reweight(X):

  (m,n)=X.shape

  colsum=np.sqrt(np.sum(X**2,axis=0))
  icol=np.argsort(-colsum)
  ## colsum+=(colsum==0)
  ## invcolsum=1/colsum
  colweight=np.ones(n)
  colweight[icol[:n-10]]=0

  ## X1=X*np.outer(np.ones(m),invcolsum)
  X1=X*np.outer(np.ones(m),colweight)
  
  return(X1)

  
## ######################################################


  




  
