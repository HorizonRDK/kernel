#!/usr/bin/perl
# ------------------------------------------------------------------------
#
#                (C) COPYRIGHT 2012 - 2016 SYNOPSYS, INC.
#                          ALL RIGHTS RESERVED
#
#  (C) COPYRIGHT 2012-2016 Synopsys, Inc. 
#  This Synopsys software and all associated documentation are 
#  proprietary to Synopsys, Inc. and may only be used pursuant 
#  to the terms and conditions of a written license agreement 
#  with Synopsys, Inc. All other use, reproduction, modification, 
#  or distribution of the Synopsys software or the associated 
#  documentation is strictly prohibited.
#
# ------------------------------------------------------------------------

################################################################################
# This script parses NIST DRBG vector rsp files, extracts "AES-128 no df" and 
# "AES-256 no df" vectors and stores them in extracted_* files.
# It searchs for all directories in the run directory and parses all the rsp 
# files in each dir.
################################################################################

use strict;
use warnings;

my @keywords = ("AES-128 no df", "AES-256 no df");

##### find all directories in side the run dir
my $run_dir = $ARGV[0];
opendir(RUN_DIR, $run_dir) or die "Cannot open the run directory $!\n";
my @dirs = grep {-d "$run_dir/$_" && ! /^\.{1,2}$/} readdir(RUN_DIR);
closedir(RUN_DIR);

# output directory
my $out_dir = "./parsed_drbgvectors";
if (!(-e $out_dir and -d $out_dir)) {
   system("mkdir $out_dir");
}

##### find rsp files in each directory
my %rsp_files;

foreach my $dir (@dirs) {   
   opendir(DIR, "$run_dir/$dir") or die "Cannot open directory $run_dir/$dir $!\n";   
   
   while (my $file = readdir(DIR)) {      
      if (($file =~ /CTR_DRBG.*\.rsp/) && ($file !~ /_parsed\.rsp/)) {
         push(@{$rsp_files{$dir}{in}}, "$run_dir/$dir/$file");
         push(@{$rsp_files{$dir}{out}}, "$out_dir/parsed_$dir\_$file");
      }
   }
   
   closedir(DIR);
}

##### parsing rsp files
foreach my $dir (keys %rsp_files) {
   my $file_cntr = 0;
   
   foreach my $file (@{$rsp_files{$dir}{in}}) {   
      print ">> parsing file $file...\n";
      open FILE, "<", "$file" or die "Cannot open file $file $!\n";
      
      # output file
      my $out_file = $rsp_files{$dir}{out}[$file_cntr];
      open OUT, ">", "$out_file" or die "Cannot open file $out_file $!\n";      
      
      my $print_open = 0;
      my $kw_str_or  = join("|", @keywords);         
      my $options = "";
      
      ### parse the file
      while (<FILE>) {
         # find all available options. this is mainly used to detect sections
         if (/#\s*CTR_DRBG options:(.*)/) {
            $options = $1;
            my @options = split("\:\:", $options);         
            $options =~ s/^\s*//; $options =~ s/\s*$//;
            $options =~ s/\s*\:\:\s*/\|/g;
         }
         # find the beggining of each section
         elsif (/$options/) {
            if (/$kw_str_or/) {
               $print_open = 1;
            } else {
               $print_open = 0;
            }
         }
         
         # print
         if ($print_open) {
            print OUT $_;
         }
      }
      
      print "-- $file parsed.\n";
      $file_cntr++;      
      close(FILE);
   }
}