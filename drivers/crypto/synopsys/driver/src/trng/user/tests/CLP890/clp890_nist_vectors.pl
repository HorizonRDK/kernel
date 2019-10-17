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

=head1 SYNOPSIS

clp890_nist_vectors.pl [-d device]

Run NIST's DRBG known answer vectors and verify the results

=cut

use warnings;
use strict;
use Testlib;

##### Variables
my $pwd = `pwd`; $pwd =~ s/\n//g;
my $vectors_dir      = $pwd . "/parsed_drbgvectors";
# if there is not input argument use default files
my $no_reseed_file = $vectors_dir . "/parsed_drbgvectors_no_reseed_CTR_DRBG.rsp";
my $pr_false_file  = $vectors_dir . "/parsed_drbgvectors_pr_false_CTR_DRBG.rsp";
my $pr_true_file   = $vectors_dir . "/parsed_drbgvectors_pr_true_CTR_DRBG.rsp";
my $dflt_ps_256;

while (my $arg = shift(@ARGV)) {
  if      ($arg eq "-no_reseed_f") {
     $no_reseed_file = $pwd . "/" . shift(@ARGV);
  } elsif ($arg eq "-pr_false_f") {
     $pr_false_file  = $pwd . "/" . shift(@ARGV);
  } elsif ($arg eq "-pr_true_f") {
     $pr_true_file   = $pwd . "/" . shift(@ARGV);
  } elsif ($arg eq "-ps") {
     $dflt_ps_256 = shift(@ARGV);
  } elsif ($arg eq "-h") {
   print "     -no_reseed_f Specify any file other than /parsed_drbgvectors_no_reseed_CTR_DRBG.rsp.\n";
   print "     -pr_false_f  Specify any file other than /parsed_drbgvectors_pr_false_CTR_DRBG.rsp.\n";
   print "     -pr_true_f   Specify any file other than /parsed_drbgvectors_pr_true_CTR_DRBG.rsp.\n";
   print "     -ps          Specify the default personalization string.\n";
   print "     -h           Shows this help menu.\n";
  } else {
     die "invalid option ($arg). User -h for help.\n";
  }
}
my $dflt_ps_128 = substr $dflt_ps_256, -64;

my %vectors;
@{$vectors{$no_reseed_file}{cmd_seq}} = ("uninstantiate", "seed", "instantiate", "gen", "gen", "done");
@{$vectors{$pr_false_file}{cmd_seq}}  = ("uninstantiate", "seed", "instantiate", "seed", "reseed", "gen", "gen", "done");
@{$vectors{$pr_true_file}{cmd_seq}}   = ("uninstantiate", "seed", "instantiate", "seed", "gen", "seed", "gen", "done");

$vectors{$no_reseed_file}{title} = "No Reseed";
$vectors{$pr_false_file}{title}  = "PR False";
$vectors{$pr_true_file}{title}   = "PR true";

my $kw_128 = "AES-128 no df";
my $kw_256 = "AES-256 no df";
my $kw_pred_resist = "PredictionResistance";
my $kw_nonce_len = "NonceLen";
my $kw_entropy_len = "EntropyInputLen";
my $kw_ps_len = "PersonalizationStringLen";
my $kw_addin_len = "AdditionalInputLen";
my $kw_ret_len = "ReturnedBitsLen";
my $kw_section = "COUNT";
my $kw_entropy = "EntropyInput";
my $kw_entropy_reseed = "EntropyInputReseed";
my $kw_entropy_pr = "EntropyInputPR";
my $kw_ps = "PersonalizationString";
my $kw_addin = "AdditionalInput";
my $kw_addin_reseed = "AdditionalInputReseed";
my $kw_ret = "ReturnedBits";

my $pred_resist;
my $sec_strength;
my $seed_len;
my $ps_exists;
my $addin_exists;
my @entropy;
my @ps;
my @addin;
my $ret;

print "/*************************************/\n";
print "/********** NIST Vector Test *********/\n";
print "/*************************************/\n";

##### Reload
if (system("rmmod $Testlib::clp890_module; insmod $Testlib::clp890_module;")) { Testlib::fail("failed to reload the module\n"); }

##### init
Testlib::init();
Testlib::sysfs_open();

##### get CLP890 features
my $state_ref = Testlib::get_clp890_features();
my %state = %{$state_ref};
print "CLP890 Features: DRBG_ARCH = $state{drbg_arch}, SECURE_RESET_STATE = $state{sec_rst_state}, EXTRA_PS_PRESENT = $state{extra_ps_present}\n";

foreach my $file (keys %vectors) {
   open FILE, "<", "$file" or die "Cannot open $file: $!\n";
   print ">>>>>>>>>>>>>>> File: $vectors{$file}{title}\n";
   
   ##### Initial step: uninstantiate and put the core in the nonce mode
   Testlib::do_uninstantiate("Initial");
   Testlib::set_nonce("on");  

   my $count = 0;
   my $blk_id = 0;
   my $blk_name;
   my $step;
   my $process_vectors = 0;
   
   while (<FILE>) {
      if (/$kw_128/) {
         print "========== \U$kw_128 - block number $blk_id\n";
         # uninstantiate and set the sec_strength to 128 bits
         Testlib::do_uninstantiate("new block");      
         Testlib::set_sec_strength(128);
         $sec_strength = 128;
         $seed_len = 256;         
         $blk_id++;
         $process_vectors = 1;
      }
      elsif (/$kw_256/ && $state{drbg_arch}) {
         print "========== \U$kw_256 - block number $blk_id\n";
         # uninstantiate and set the sec_strength to 256 bits
         Testlib::do_uninstantiate("new block");      
         Testlib::set_sec_strength(256);
         $sec_strength = 256;
         $seed_len = 384;
         $blk_id++;         
         $process_vectors = 1;
      }
      if ($process_vectors) {
         if (/$kw_pred_resist\s*=\s*(.*)\s*]/) {
            if ($1 eq "True") {
               $pred_resist = 1;
            } elsif ($1 eq "False") {
               $pred_resist = 0;
            } else {
               print "ERROR: incorrect value for $kw_pred_resist: $1\n";
            }
         }
         elsif (/$kw_entropy_len\s*=\s*(.*)\s*]/) {
            if ($1 != $seed_len) {
               print "ERROR: $kw_entropy_len has to be $seed_len, not $1\n";
            }
         }
         elsif (/$kw_nonce_len\s*=\s*(.*)\s*]/) {
            if ($1 != 0) {
               print "ERROR: $kw_nonce_len has to be 0, not $1\n";
            }
         }
         elsif (/$kw_ps_len\s*=\s*(.*)\s*]/) {
            if ($1 == 0) {
               $ps_exists = 0;
            } elsif ($1 != $seed_len) {
               print "ERROR: $kw_ps_len has to be $seed_len, not $1\n";
            } else {
               $ps_exists = 1;
               # if HW does not support extra personilization string, skip this section of the vector
               if (!$state{extra_ps_present}) {
                  print ">> SKIP this vector block, because it has personilization string, but HW is not configured to accept extra personilization string.\n";
                  $process_vectors = 0;
               }
            }
         }
         elsif (/$kw_addin_len\s*=\s*(.*)\s*]/) {
            if ($1 == 0) {
               $addin_exists = 0;
            } elsif ($1 != $seed_len) {
               print "ERROR: $kw_addin_len has to be $seed_len, not $1\n";
            } else {
               $addin_exists = 1;
            }
         }
         elsif (/$kw_ret_len\s*=\s*(.*)\s*]/) {
            if ($1 != 512) {
               print "ERROR: $kw_ret_len has to be 512, not $1\n";
            }
            
            print ">> Step $count: Addin = $addin_exists --- PS = $ps_exists\n";
         }
         elsif (/$kw_section\s*=\s*([\d]+)\s*/) {
            $count = $1;
         }
         elsif (/$kw_entropy\s*=\s*([0-9,a-f]+)/ || /$kw_entropy_reseed\s*=\s*([0-9,a-f]+)/ || /$kw_entropy_pr\s*=\s*([0-9,a-f]+)/) {
            push(@entropy, $1);
         }
         elsif (/$kw_ps\s*=\s*([0-9,a-f]+)/) {
            if ($sec_strength == 128) {
               my $xor = unpack('h*', (pack('h*', $1) ^ pack('h*', $dflt_ps_128)));
               push(@ps, $xor);
            } else {                        
               my $xor = unpack('h*', (pack('h*', $1) ^ pack('h*', $dflt_ps_256)));               
               push(@ps, $xor);
            }
         }
         elsif (/$kw_ps\s*=/) {
            if ($sec_strength == 128) {
               push(@ps, $dflt_ps_128);
            } else {
               push(@ps, $dflt_ps_256);
            }
         }
         elsif (/$kw_addin\s*=\s*([0-9,a-f]+)/ || /$kw_addin_reseed\s*=\s*([0-9,a-f]+)/) {
            push(@addin, $1);
         }
         elsif (/$kw_ret\s*=\s*([0-9,a-f]+)/) {
            $ret = $1;
   
            ### all information is gathered, perform the command sequence
            foreach my $cmd (@{$vectors{$file}{cmd_seq}}) {         
               if ($cmd eq "uninstantiate") {
                  Testlib::do_uninstantiate($count);
               } elsif ($cmd eq "seed") {
                  Testlib::do_load_seed_directly($count, shift(@entropy));
               } elsif ($cmd eq "instantiate") {
                  Testlib::do_instantiate($count, $sec_strength, $pred_resist, 0, shift(@ps));
               } elsif ($cmd eq "gen") {
                  Testlib::do_generate($count, $sec_strength, $pred_resist, 0, 1, shift(@addin));
               } elsif ($cmd eq "reseed") {
                  Testlib::do_reseed($count, $pred_resist, 0, shift(@addin));
               } elsif ($cmd eq "done") {
                  my $rand_out = Testlib::get_rand_out();
                  if ($rand_out ne $ret) {
                     Testlib::fail("random ouputs don't match.\nCLP-890: $rand_out\nNIST   : $ret\n");
                  } else {
                     print ">> Step $count: Comparison succeed!\n";
                  }
               } else {
                  print "ERROR: Invalid command.\n";
               }
            }
         }
      }
   }
   
   close FILE;
}

Testlib::pass();
