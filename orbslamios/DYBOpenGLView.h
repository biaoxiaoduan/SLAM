//
//  DYBOpenGLView.h
//  orbslamios
//
//  Created by 段炎彪 on 05/05/2017.
//  Copyright © 2017 Ying Gaoxuan. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface DYBOpenGLView : UIView

@property (nonatomic , assign) BOOL isFullYUVRange;

- (void)setupGL;
- (void)displayPixelBuffer:(CVPixelBufferRef)pixelBuffer;

@end
