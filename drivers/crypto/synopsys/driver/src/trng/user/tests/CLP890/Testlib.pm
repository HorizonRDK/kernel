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

package Testlib;

use strict;

use Getopt::Long;
use Pod::Find qw(pod_where);
use Pod::Usage;
use IO::File;

use Exporter;
our ($VERSION, @ISA, @EXPORT);

$VERSION = 0;
@ISA     = qw(Exporter);
@EXPORT  = qw();

our $device = "clp890";
our $result = undef;
our $quiet = 0;
our @handles;
our $clp890_module = "../../../../../bin";$clp890_module = `cd "$clp890_module"; pwd`;chomp($clp890_module);$clp890_module .= "/elpclp890.ko";
our $MAX_REPEAT = 50;

##### attributes
our $attr_features       = "features";
our $attr_sec_strength   = "sec_strength";
our $attr_nonce          = "nonce";
our $attr_mode           = "mode_reg";
our $attr_secure         = "secure";
our $attr_generate       = "generate";
our $attr_rand_reg       = "rand_reg";
our $attr_seed_reg       = "seed_reg";
our $attr_npa_reg        = "npa_data_reg";
our $attr_hw_state       = "hw_state";
our $attr_reseed         = "reseed";
our $attr_uninstantiate  = "uninstantiate";
our $attr_instantiate    = "instantiate";
our $attr_seed_direct    = "nonce_seed_direct";
our $attr_rand_out       = "rand_out";
our $attr_cmd_reg        = "ctrl_reg";
our $attr_max_bits_p_req = "max_bits_per_req";
our $attr_max_req_p_seed = "max_req_per_seed";
our $attr_kat            = "kat";

# field masks
our $msk_pred_resist = 0x00000008;
our $msk_sec_strength = 0x00000001;

sub print_version {
   print "clp890-tests 1.0\n";
}

sub print_help {
   print_version();
   pod2usage( { -verbose => 99,
                -exitval => "NOEXIT",
                -section => [ qw/SYNOPSIS OPTIONS/ ]} );
   pod2usage( { -input => pod_where({-inc => 1}, __PACKAGE__),
                -verbose => 99,
                -exitval => "NOEXIT",
                -section => [ qw/OPTIONS/ ]}, );
}

=head1 GLOBAL OPTIONS

=over 8

=item B<-d>, B<--device>=F<device>

Specify the platform device name (default: clp890).

=item B<-q>, B<--quiet>

Do not print PASSED on successful test runs.  Failure messages are still
printed.

=back

=cut
sub init {
   Getopt::Long::Configure('gnu_getopt', 'auto_help');
   GetOptions(
      'd|device=s' => \$device,
      'q|quiet'    => \$quiet,
      'V|version'  => sub { print_version(); $result = 0; exit },
      'H|help'     => sub { print_help();    $result = 0; exit },
      @_,
   ) or die;
}

sub pass {
   print STDOUT "PASSED\n" unless ($quiet);
   $result = 0;
   exit 0;
}

sub fail {
   my ($msg) = @_;

   chomp($msg);
   print STDOUT "FAILED: $msg\n";
   $result = 1;
   exit 1;
}

END {
   die "FAILED to complete test\n" unless defined $result;
}

sub sysfs_open {
   chdir "/sys/bus/platform/devices/$device"
      or die "failed to open $device: $!\n";  
}

sub open_attribute {
   my ($filename, $mode) = @_;
   my %attr;

   $attr{filename} = $filename;
   $attr{handle} = new IO::File($filename, $mode)
      or die "failed to open $filename: $!\n";

   return push(@handles, (\%attr)) - 1;
}

sub close_attribute {
   my ($index) = @_;
   my $attr = $handles[$index];   
   
   die "invalid handle: $index\n" unless defined($attr);
   
   close $attr->{handle};
}

sub read_attribute {
   my ($index) = @_;
   my $attr = $handles[$index];
   my ($handle, $filename);

   die "invalid handle: $index\n" unless defined($attr);

   $handle = $attr->{handle};
   $filename = $attr->{filename};

   $handle->seek(0, 0) or die "failed to rewind $filename: $!\n";
   $_ = <$handle> or die "failed to read $filename: $!\n";

   chomp;
   return $_;
}

sub write_attribute {
   my ($index, $data) = @_;
   my $attr = $handles[$index];
   my ($handle, $filename);

   die "invalid handle: $index\n" unless defined($attr);

   $handle = $attr->{handle};
   $filename = $attr->{filename};

   $handle->printflush($data)
      or return -1;
}

sub write_cmd_reg {
   my ($cmd_reg) = @_;
   
   my $ret;
   
   my $wr_cmd_reg = open_attribute($attr_cmd_reg, ">");
   if (($ret = write_attribute($wr_cmd_reg, $cmd_reg)) > 0) {
      close_attribute($wr_cmd_reg) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   } else {
      Testlib::fail("Unsuccessfull write to $attr_cmd_reg\n");
   }
   
   return $ret;
}

sub write_kat {
   my ($kat) = @_;
   
   my $ret;
   
   my $wr_kat = open_attribute($attr_kat, ">");
   if (($ret = write_attribute($wr_kat, $kat)) > 0) {
      close_attribute($wr_kat) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   } else {
      Testlib::fail("Unsuccessfull write to $attr_kat\n");
   }
   
   return $ret;
}

sub write_npa_reg {
   my ($npa_reg) = @_;
   
   my $ret;
   
   my $wr_npa_reg = open_attribute($attr_npa_reg, ">");
   if (($ret = write_attribute($wr_npa_reg, $npa_reg)) > 0) {
      close_attribute($wr_npa_reg) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   } else {
      Testlib::fail("Unsuccessfull write to $attr_npa_reg\n");
   }
   
   return $ret;
}

sub write_seed_reg {
   my ($seed_reg) = @_;
   
   my $ret;
   
   my $wr_seed_reg = open_attribute($attr_seed_reg, ">");
   if (($ret = write_attribute($wr_seed_reg, $seed_reg)) > 0) {
      close_attribute($wr_seed_reg) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   } else {
      Testlib::fail("Unsuccessfull write to $attr_seed_reg\n");
   }
   
   return $ret;
}

sub get_clp890_features {
   my %state;

   my $rd_features = open_attribute($attr_features, "<");
   my $str_features = read_attribute($rd_features);
   
   my $drbg_arch; my $sec_rst_state; my $extra_ps_present;
   if ($str_features =~ /drbg_arch\s*=\s*([\d+])\s*,/) {
      $state{drbg_arch} = $1;
   }
   if ($str_features =~ /secure_rst_state\s*=\s*([\d+])\s*,/) {
      $state{sec_rst_state} = $1;
   }
   if ($str_features =~ /extra_ps_present\s*=\s*([\d+])\s*/) {
      $state{extra_ps_present} = $1;
   }
   
   close_attribute($rd_features) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   
   return \%state;
}

sub set_max_bits_p_req {
   my ($max_bits_p_req) = @_;
   
   my $ret;

   my $wr_max_bits_p_req = open_attribute($attr_max_bits_p_req, ">");
   if (($ret = write_attribute($wr_max_bits_p_req, $max_bits_p_req)) > 0) {
      close_attribute($wr_max_bits_p_req) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
   return $ret;
}

sub get_max_bits_p_req {
   my $rd_max_bits_p_req = open_attribute($attr_max_bits_p_req, "<");
   my $max_bits_p_req = read_attribute($rd_max_bits_p_req);
   close_attribute($rd_max_bits_p_req) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return $max_bits_p_req;
}

sub set_max_req_p_seed {
   my ($max_req_p_seed) = @_;
   
   my $ret;
   
   my $wr_max_req_p_seed = open_attribute($attr_max_req_p_seed, ">");
   if (($ret = write_attribute($wr_max_req_p_seed, $max_req_p_seed)) > 0) {
      close_attribute($wr_max_req_p_seed) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
   return $ret;
}

sub get_max_req_p_seed {
   my $rd_max_req_p_seed = open_attribute($attr_max_req_p_seed, "<");
   my $max_req_p_seed = read_attribute($rd_max_req_p_seed);
   close_attribute($rd_max_req_p_seed) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return $max_req_p_seed;
}

sub set_nonce {
   my ($nonce) = @_;
   
   my $ret;
   
   my $wr_nonce = open_attribute($attr_nonce, ">");
   if (($ret = write_attribute($wr_nonce, $nonce)) > 0) {
      close_attribute($wr_nonce) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
   return $ret;
}

sub set_secure {
   my ($secure) = @_;
   
   my $ret;
   
   my $wr_secure = open_attribute($attr_secure, ">");
   if (($ret = write_attribute($wr_secure, $secure)) > 0) {   
      close_attribute($wr_secure) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
   return $ret;
}

sub set_sec_strength {
   my ($sec_strength) = @_;
   
   my $ret;
   
   my $wr_sec_strength = open_attribute($attr_sec_strength, ">");
   if (($ret = write_attribute($wr_sec_strength, $sec_strength)) > 0) {   
      close_attribute($wr_sec_strength) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
   return $ret;
}

sub get_sec_strength {
   my $rd_mode = open_attribute($attr_mode, "<");
   my $mode_reg = read_attribute($rd_mode);
   close_attribute($rd_mode) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return (($mode_reg & $msk_sec_strength) ? 256 : 128);
}

sub get_hw_pred_resist {
   my $rd_mode = open_attribute($attr_mode, "<");
   my $mode_reg = read_attribute($rd_mode);
   close_attribute($rd_mode) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return (($mode_reg & $msk_pred_resist) ? 1 : 0);
}

sub sec_strength_string
{
   my ($sec_strength) = @_;
   
   return ($sec_strength == 128) ? "0" : ($sec_strength == 256) ? "1" : die "Invalid security strength ($sec_strength)";
}

sub get_rand_out {
   my $rd_rand_out = open_attribute($attr_rand_out, "<");
   my $rand_out = read_attribute($rd_rand_out);
   close_attribute($rd_rand_out) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return $rand_out;
}

sub get_seed_reg {
   my $rd_seed = open_attribute($attr_seed_reg, "<");
   my $seed = read_attribute($rd_seed);
   close_attribute($rd_seed) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return $seed;
}

sub get_npa_reg {
   my $rd_npa = open_attribute($attr_npa_reg, "<");
   my $npa = read_attribute($rd_npa);
   close_attribute($rd_npa) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return $npa;
}

sub get_hw_state {
   my $rd_hw_state = open_attribute($attr_hw_state, "<");
   
   my $attr = $handles[$rd_hw_state];
   my ($handle, $filename);

   die "invalid handle: $rd_hw_state\n" unless defined($attr);

   $handle = $attr->{handle};
   $filename = $attr->{filename};

   $handle->seek(0, 0) or die "failed to rewind $filename: $!\n";
   $_ = <$handle> or die "failed to read $filename: $!\n";
   chomp;   
   my $key = $_;
   $key =~ s/Key\s*=\s*//;   

   $handle->seek(0, 1) or die "failed to rewind $filename: $!\n";
   $_ = <$handle> or die "failed to read $filename: $!\n";
   chomp;
   my $v = $_;
   $v =~ s/V\s*=\s*//;   
   
   close_attribute($rd_hw_state) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   return ($key, $v);
}

sub do_generate {
   my ($step, $sec_strength, $pred_resist, $error, $rpt, $addin) = @_;   
   
   # if rpt is not specified, randomize it
   if (!$rpt) {
      $rpt = rand_num();
   }
   my $options = $pred_resist . Testlib::sec_strength_string($sec_strength) . ($addin ? "1-$addin" : "0");
   my $rand_out = 0;
   my $prev_rand_out = 0;
   my $ret = 0;
   
   print ">> Step $step: Generate $rpt times - security strength = $sec_strength, pred_resist = $pred_resist - process must " . ($error ? "return error" : "succeed") . "\n";
   
   for (my $i = 0; $i < $rpt; $i++) {
      my $wr_generate = Testlib::open_attribute($Testlib::attr_generate, ">");
   
      ### Generate must succeed      
      if (!$error) {
         if (($ret = Testlib::write_attribute($wr_generate, $options)) > 0) {
            my $rd_rand_reg = Testlib::open_attribute($Testlib::attr_rand_reg, "<");
            $rand_out = Testlib::read_attribute($rd_rand_reg);
      
            # if random output is all zeros (512 bits) or it's equal to the last random output, Generate must have been failed.
            if ($rand_out eq "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000") {
               Testlib::fail("Step $step, repeat $i: Random output is 0");
            } elsif (($i != 0) && ($rand_out eq $prev_rand_out)) {
               Testlib::fail("Step $step, repeat $i: Random output has not changed from last generate. Generate process must have failed.");
            }
      
            $prev_rand_out = $rand_out;
            close_attribute($rd_rand_reg) or die "Cannot close attribute at line __LINE__: $!\n";
         } else {
            Testlib::fail("Step $step, repeat $i: Generate failed ($ret).\n");
         }
         close_attribute($wr_generate) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
      }
      ### Generate must return error
      else {
         if (($ret = Testlib::write_attribute($wr_generate, $options)) < 0) {
            my $rd_rand_reg = Testlib::open_attribute($Testlib::attr_rand_reg, "<");
            $rand_out = Testlib::read_attribute($rd_rand_reg);
            # random output must be all zeros (512 bits)
            if (($i != 0) && ($rand_out ne $prev_rand_out)) {
               Testlib::fail("Step $step, repeat $i: Random output is changed");
            }
            
            $prev_rand_out = $rand_out;
            close_attribute($rd_rand_reg) or die "Cannot close attribute at line __LINE__: $!\n";
         } else {
            Testlib::fail("Step $step, repeat $i: Generate did not return error ($ret).\n");
         }
      }      
   } # for loop     
} # call_generate_rand_times

sub do_reseed {
   my ($step, $pred_resist, $error, $addin) = @_;

   my $options = $pred_resist . ($addin ? "1-$addin" : "0");
   my $ret = 0;

   print ">> Step $step: Reseed - pred_resist = $pred_resist - process must " . ($error ? "return error" : "succeed") . "\n";
   
   if (!$error) {
      my $wr_reseed = Testlib::open_attribute($Testlib::attr_reseed, ">");
      if (($ret = Testlib::write_attribute($wr_reseed, $options)) < 0) {
         Testlib::fail("Step $step: Reseed failed ($ret)\n");
      }
      
      close_attribute($wr_reseed) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
   }
   
} # do_reseed_rand_times

sub do_uninstantiate {
   my ($step) = @_;
   
   my $ret;
   
   print ">> Step $step: Uninstantiate\n";
   
   my $wr_uninstantiate = Testlib::open_attribute($Testlib::attr_uninstantiate, ">");
   if (($ret = Testlib::write_attribute($wr_uninstantiate, "0")) < 0) {
      Testlib::fail("Step $step: Uninstantiate failed ($ret)\n");
   }
   
   close_attribute($wr_uninstantiate) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
}

sub do_instantiate {
   my ($step, $sec_strength, $pred_resist, $error, $ps) = @_;

   my $options = $pred_resist . Testlib::sec_strength_string($sec_strength) . ($ps ? "1-$ps" : "0");
   my $ret;
   
   print ">> Step $step: Instantiate - security strength = $sec_strength, pred_resist = $pred_resist\n";
   
   my $wr_instantiate = Testlib::open_attribute($Testlib::attr_instantiate, ">");
   
   if (!$error) {
      if (($ret = Testlib::write_attribute($wr_instantiate, $options)) < 0) {
         Testlib::fail("Step $step: Instantiate failed ($ret)\n");
      }   
   }
   
   close_attribute($wr_instantiate) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
}

sub do_load_seed_directly {
   my ($step, $seed) = @_;
   
   my $ret;
   
   print ">> Step $step: Load seed directly\n";
   
   my $wr_seed_direct = Testlib::open_attribute($Testlib::attr_seed_direct, ">");
      
   if (($ret = Testlib::write_attribute($wr_seed_direct, $seed)) < 0) {
      Testlib::fail("Step $step: Load seed_directly failed ($ret)\n");
   }
   
   close_attribute($wr_seed_direct) or die "Cannot close attribute in file ", __FILE__, " at line ", __LINE__, ": $!\n";
}

sub rand_num {
   return int(rand($MAX_REPEAT)) + 1;
}

1;
